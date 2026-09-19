#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""제어부: SEARCH(제자리 yaw 회전 탐색) -> APPROACH(P제어 정렬·접근) -> HOLD
상태머신. px4_msgs 오프보드로 명령한다. APPROACH/HOLD에서 타겟을 놓치면 곧바로
SEARCH(전체 회전)로 튀지 않고 REACQUIRE(제자리 대기 -> 좁은 스윕 -> 그래도
안되면 SEARCH 승격)를 거친다 (_handle_lost 참고).

안전 브링업 단계는 bringup_level 파라미터로 순서대로 검증한다:
  0 LOG_ONLY     : 오프보드 미발행/미arm. 변환된 목표 NED만 로그 (변환 검증).
  1 HOVER_HOLD   : arm+offboard, 제자리 유지만 (탐색/접근 비활성).
  2 YAW          : + 탐색 회전, 접근 시 yaw축만 서보 (위치 고정).
  3 YAW_LATERAL  : + lateral축도 서보 (forward는 고정, standoff 접근 없음).
  4 FULL         : forward 포함 전체 (표준 접근 시퀀스).

레벨과 무관하게 LOG_ONLY는 항상 레벨 4 기준으로 계산만 하고 로그만 남긴다
(변환 로직 자체를 검증하기 위함)."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleAttitude,
)
from geometry_msgs.msg import PoseStamped

from kiosk_vision.wall_geometry import quat_to_rotmat, yaw_err_from_R, wrap_deg_diff

LOG_ONLY, HOVER_HOLD, YAW, YAW_LATERAL, FULL = range(5)
LEVEL_NAMES = ['LOG_ONLY', 'HOVER_HOLD', 'YAW', 'YAW_LATERAL', 'FULL']

CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
ARM_TICK = 10  # 예제(offboard_control.py)와 동일하게 setpoint 10회 스트리밍 후 arm


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ApproachControlNode(Node):
    def __init__(self):
        super().__init__('approach_control_node')
        self.declare_parameter('bringup_level', LOG_ONLY)
        self.declare_parameter('pose_topic', '/target/pose')
        # 0.6m -> 1.0m: 1m 벽 패널 기준으로도 접근 끝(정지 지점)에서 4마커가 화각에
        # 남아 aruco_pnp_node의 마커 수 게이트(REQUIRED_MARKERS=4)가 막판에 끊기지
        # 않도록 여유를 둠.
        self.declare_parameter('standoff', 1.0)
        self.declare_parameter('search_yaw_rate_deg', 20.0)
        self.declare_parameter('approach_gain', 0.3)
        # yaw도 다른 축처럼 P제어로: "전량 적용"은 과단순화였음 (단일 프레임 노이즈가
        # 감쇠 없이 그대로 명령에 반영되어 진동의 원인이 됐음, docs/PROGRESS.md 참고).
        self.declare_parameter('yaw_gain', 0.3)
        # solvePnP 거울해(flip) 사이 chatter 완화용 EMA 저역통과.
        self.declare_parameter('ema_alpha', 0.3)
        # yaw 명령 자체도 급변 못 하게 rate-limit (탐색 회전의 search_yaw_rate_deg와는
        # 별개 — 이건 APPROACH/HOLD의 비전 기반 yaw 보정에만 적용).
        self.declare_parameter('yaw_rate_limit_deg', 15.0)
        self.declare_parameter('max_speed', 0.3)  # m/s
        self.declare_parameter('tol_forward', 0.06)
        self.declare_parameter('tol_lateral', 0.06)
        self.declare_parameter('tol_yaw_deg', 3.0)
        self.declare_parameter('takeoff_alt', 1.5)  # m (NED z = -takeoff_alt)
        # 실측 카메라 프레임 간격이 5Hz 스펙보다 불규칙(WSL 렌더링, 최대 약 2.0s 공백
        # 관측됨)해서 여유를 두고 3.0s로 설정 (미세 드롭이 REACQUIRE로 안 튀도록 소폭 상향).
        self.declare_parameter('target_lost_timeout', 3.0)  # s
        # REACQUIRE(재포착): 락을 막 놓쳤을 때 곧바로 반대 방향 전체 SEARCH로 튀지
        # 않기 위한 3단계 — ①grace(제자리 대기) ②좁은 스윕(마지막 방향 근방) ③그래도
        # 못 찾으면 전체 SEARCH로 승격.
        self.declare_parameter('reacquire_grace_s', 1.75)
        self.declare_parameter('reacquire_sweep_deg', 40.0)
        self.declare_parameter('reacquire_sweep_rate_deg', 9.0)
        self.declare_parameter('reacquire_sweep_max_s', 15.0)

        self.level = int(self.get_parameter('bringup_level').value)
        self.standoff = float(self.get_parameter('standoff').value)
        self.search_yaw_rate = math.radians(float(self.get_parameter('search_yaw_rate_deg').value))
        self.gain = float(self.get_parameter('approach_gain').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.yaw_rate_limit = math.radians(float(self.get_parameter('yaw_rate_limit_deg').value))
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.tol_forward = float(self.get_parameter('tol_forward').value)
        self.tol_lateral = float(self.get_parameter('tol_lateral').value)
        self.tol_yaw = math.radians(float(self.get_parameter('tol_yaw_deg').value))
        self.takeoff_alt = float(self.get_parameter('takeoff_alt').value)
        self.target_lost_timeout = float(self.get_parameter('target_lost_timeout').value)
        self.reacquire_grace_s = float(self.get_parameter('reacquire_grace_s').value)
        self.reacquire_sweep = math.radians(float(self.get_parameter('reacquire_sweep_deg').value))
        self.reacquire_sweep_rate = math.radians(float(self.get_parameter('reacquire_sweep_rate_deg').value))
        self.reacquire_sweep_max_s = float(self.get_parameter('reacquire_sweep_max_s').value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                  self.on_local_position, px4_qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                  self.on_attitude, px4_qos)

        pose_topic = self.get_parameter('pose_topic').value
        self.create_subscription(PoseStamped, pose_topic, self.on_target_pose, 10)

        self.vlp = None
        self.att = None
        self.last_target_pose = None
        self.last_target_time = None
        self.ema_lateral = None
        self.ema_yaw_err_deg = None

        self.state = 'SEARCH'
        self.search_ned = None
        self.yaw_ref = None
        self.target_ned = None
        self.target_yaw = None
        self.offboard_setpoint_counter = 0

        self._reacquire_since = None
        self._reacquire_center_yaw = None
        self._reacquire_hold_ned = None
        self._reacquire_sweep_dir = 1.0

        self.create_timer(DT, self.on_timer)
        self.get_logger().info(
            f'approach_control_node start bringup_level={self.level}({LEVEL_NAMES[self.level]}) '
            f'standoff={self.standoff}m gain={self.gain} yaw_gain={self.yaw_gain} '
            f'yaw_rate_limit={math.degrees(self.yaw_rate_limit):.0f}deg/s ema_alpha={self.ema_alpha} '
            f'max_speed={self.max_speed}m/s')

    # ---- subscriptions ----
    def on_local_position(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.vlp = msg

    def on_attitude(self, msg):
        self.att = msg

    def on_target_pose(self, msg):
        self.last_target_pose = msg
        self.last_target_time = self.get_clock().now()

        raw_lateral = msg.pose.position.x
        R = quat_to_rotmat(msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w)
        raw_yaw_err_deg = yaw_err_from_R(R)
        if self.ema_lateral is None:
            self.ema_lateral = raw_lateral
            self.ema_yaw_err_deg = raw_yaw_err_deg
        else:
            a = self.ema_alpha
            self.ema_lateral = a * raw_lateral + (1.0 - a) * self.ema_lateral
            # 각도는 선형 평균하면 ±180deg 경계에서 깨짐 -> wrap된 델타로 갱신
            self.ema_yaw_err_deg += a * wrap_deg_diff(raw_yaw_err_deg, self.ema_yaw_err_deg)

    # ---- helpers ----
    def current_yaw(self):
        w, x, y, z = self.att.q  # Hamilton, FRD body -> NED, order (w,x,y,z)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def target_lost(self):
        # 프레임 단위 순간 미검출(self.target_visible=False)만으로 즉시 SEARCH로 튀지
        # 않도록, "마지막으로 pose를 받은 시각"만 기준으로 판단한다 (target_lost_timeout
        # 동안은 마지막 pose를 유지). 카메라 프레임 간격이 불규칙(WSL 렌더링)해서
        # self.target_visible을 직접 쓰면 수백ms 공백마다 SEARCH<->APPROACH가 토글됨.
        if self.last_target_time is None:
            return True
        age = (self.get_clock().now() - self.last_target_time).nanoseconds * 1e-9
        return age > self.target_lost_timeout

    def clamp_step(self, cur_n, cur_e, tgt_n, tgt_e):
        dn, de = tgt_n - cur_n, tgt_e - cur_e
        dist = math.hypot(dn, de)
        max_step = self.max_speed * DT
        if dist > max_step and dist > 1e-9:
            k = max_step / dist
            return cur_n + dn * k, cur_e + de * k
        return tgt_n, tgt_e

    # ---- main loop ----
    def on_timer(self):
        if self.vlp is None or self.att is None:
            return

        cur_n, cur_e, cur_d = self.vlp.x, self.vlp.y, self.vlp.z
        cur_yaw = self.current_yaw()

        if self.search_ned is None:
            self.search_ned = (cur_n, cur_e, -self.takeoff_alt)
            self.yaw_ref = cur_yaw
            self.target_ned = self.search_ned
            self.target_yaw = self.yaw_ref

        if self.level == HOVER_HOLD:
            self.state = 'SEARCH'
            target_ned, target_yaw = self.search_ned, self.yaw_ref
        else:
            target_ned, target_yaw = self.run_state_machine(cur_n, cur_e, cur_yaw)

        raw_target_ned = target_ned  # 속도캡 적용 전 (실비행 시 여러 틱에 걸쳐 이 방향으로 수렴)
        # 급가속 방지: 실제 현재 위치 기준으로 한 틱당 이동을 max_speed*DT로 제한
        clamped_n, clamped_e = self.clamp_step(cur_n, cur_e, target_ned[0], target_ned[1])
        target_ned = (clamped_n, clamped_e, target_ned[2])
        self.target_ned, self.target_yaw = target_ned, target_yaw

        if self.level == LOG_ONLY:
            self.get_logger().info(
                f'[LOG_ONLY][{self.state}] cur_ned=({cur_n:.3f},{cur_e:.3f},{cur_d:.3f}) yaw={math.degrees(cur_yaw):+.1f}deg '
                f'-> raw_target_ned(속도캡 전)=({raw_target_ned[0]:.3f},{raw_target_ned[1]:.3f},{raw_target_ned[2]:.3f}) '
                f'clamped_target_ned=({target_ned[0]:.3f},{target_ned[1]:.3f},{target_ned[2]:.3f}) '
                f'target_yaw={math.degrees(target_yaw):+.1f}deg',
                throttle_duration_sec=0.5)
            return  # 오프보드 발행/arm 없음 — 드론은 움직이지 않는다

        self.publish_offboard_heartbeat()
        self.publish_trajectory_setpoint(target_ned, target_yaw)

        if self.offboard_setpoint_counter == ARM_TICK:
            self.engage_offboard_mode()
            self.arm()
        if self.offboard_setpoint_counter <= ARM_TICK:
            self.offboard_setpoint_counter += 1

        self.get_logger().info(
            f'[{LEVEL_NAMES[self.level]}][{self.state}] target_ned=({target_ned[0]:.3f},{target_ned[1]:.3f},{target_ned[2]:.3f}) '
            f'target_yaw={math.degrees(target_yaw):+.1f}deg', throttle_duration_sec=0.5)

    def _handle_lost(self):
        """타겟 유실 시 곧바로 전체 SEARCH로 튀지 않고 REACQUIRE 3단계를 거친다:
        ①grace(제자리 대기) ②마지막 방향 근방 좁은 스윕 ③그래도 못 찾으면 전체 SEARCH."""
        now = self.get_clock().now()

        if self.state in ('APPROACH', 'HOLD'):
            self.state = 'REACQUIRE'
            self._reacquire_since = now
            self._reacquire_center_yaw = self.target_yaw
            self._reacquire_hold_ned = self.target_ned
            self._reacquire_sweep_dir = 1.0
            self.yaw_ref = self.target_yaw
            self.get_logger().info('타겟 유실 -> REACQUIRE(제자리 대기) 진입')

        if self.state == 'REACQUIRE':
            elapsed = (now - self._reacquire_since).nanoseconds * 1e-9
            if elapsed <= self.reacquire_grace_s:
                return self._reacquire_hold_ned, self._reacquire_center_yaw  # ① grace: 제자리 대기

            if elapsed <= self.reacquire_grace_s + self.reacquire_sweep_max_s:
                # ② 마지막 방향 기준 좁은 스윕 (느린 속도로 왕복)
                step = self.reacquire_sweep_rate * DT * self._reacquire_sweep_dir
                candidate = wrap_pi(self.yaw_ref + step)
                offset = wrap_pi(candidate - self._reacquire_center_yaw)
                if offset > self.reacquire_sweep:
                    candidate = wrap_pi(self._reacquire_center_yaw + self.reacquire_sweep)
                    self._reacquire_sweep_dir = -1.0
                elif offset < -self.reacquire_sweep:
                    candidate = wrap_pi(self._reacquire_center_yaw - self.reacquire_sweep)
                    self._reacquire_sweep_dir = 1.0
                self.yaw_ref = candidate
                return self._reacquire_hold_ned, self.yaw_ref

            # ③ 좁은 스윕도 실패 -> 전체 SEARCH로 승격 (기존 동작, search_ned로 복귀)
            self.get_logger().info('REACQUIRE 실패(좁은 스윕 시간초과) -> 전체 SEARCH 회전으로 전환')
            self.state = 'SEARCH'
            self.yaw_ref = self.target_yaw

        self.yaw_ref = wrap_pi(self.yaw_ref + self.search_yaw_rate * DT)
        return self.search_ned, self.yaw_ref

    def run_state_machine(self, cur_n, cur_e, cur_yaw):
        if self.target_lost():
            return self._handle_lost()

        forward = self.last_target_pose.pose.position.z  # forward는 스무딩 없이 원값 사용
        lateral = self.ema_lateral
        yaw_err = math.radians(self.ema_yaw_err_deg)

        forward_err = forward - self.standoff
        lateral_err = lateral

        # LOG_ONLY는 실제로는 아무것도 안 움직이지만 변환식 검증을 위해 항상
        # FULL 기준으로 계산한다(로그만 남기고 오프보드는 발행하지 않음).
        gate_level = FULL if self.level == LOG_ONLY else self.level
        active_forward = forward_err if gate_level >= FULL else 0.0
        active_lateral = lateral_err if gate_level >= YAW_LATERAL else 0.0

        dn = active_forward * math.cos(cur_yaw) - active_lateral * math.sin(cur_yaw)
        de = active_forward * math.sin(cur_yaw) + active_lateral * math.cos(cur_yaw)
        candidate_ned = (cur_n + self.gain * dn, cur_e + self.gain * de, self.search_ned[2])
        yaw_step = self.yaw_gain * yaw_err
        max_yaw_step = self.yaw_rate_limit * DT
        yaw_step = max(-max_yaw_step, min(max_yaw_step, yaw_step))
        candidate_yaw = wrap_pi(cur_yaw + yaw_step)

        checks = [abs(yaw_err) < self.tol_yaw]
        if gate_level >= YAW_LATERAL:
            checks.append(abs(lateral_err) < self.tol_lateral)
        if gate_level >= FULL:
            checks.append(abs(forward_err) < self.tol_forward)
        within_tol = all(checks)

        if within_tol:
            if self.state != 'HOLD':
                self.state = 'HOLD'
                self.target_ned, self.target_yaw = candidate_ned, candidate_yaw
            return self.target_ned, self.target_yaw

        self.state = 'APPROACH'
        return candidate_ned, candidate_yaw

    # ---- px4 command helpers (offboard_control.py 예제와 동일 패턴) ----
    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, ned, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(ned[0]), float(ned[1]), float(ned[2])]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')


def main(args=None):
    rclpy.init(args=args)
    node = ApproachControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
