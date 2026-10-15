#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""제어부: SEARCH(제자리 yaw 회전 탐색) -> APPROACH(P제어 정렬·접근) -> HOLD
상태머신. px4_msgs 오프보드로 명령한다. APPROACH/HOLD에서 타겟을 놓치면 곧바로
SEARCH(전체 회전)로 튀지 않고 REACQUIRE(제자리 대기 -> 좁은 스윕 -> 그래도
안되면 SEARCH 승격)를 거친다 (_handle_lost 참고).

IBVS식 상대속도 서보잉(SCRUM-22): 수평 절대위치(NED x,y) 없이 카메라가 본 타겟
상대오차(전진거리/좌우오프셋/yaw오차)만으로 기체(FRD) 속도 + yaw rate를 만든다 —
GPS 없는 실내 대응. 수평은 velocity, 고도만 position(z=-takeoff_alt)인 혼합
setpoint로 발행한다(고도는 baro/거리센서로 로컬 관측되는 다른 성질이라 유지).

2-시계 구조: decide()는 비전 프레임 도착 시(≈2Hz)에만 속도 명령을 계산해 저장하고,
PX4 발행은 20Hz 타이머(on_setpoint_timer)가 최신 명령을 들고 반복 발행한다 — 2Hz로
속도를 PX4에 직접 던지면 정지 진동이 재발한다(docs/PROGRESS.md velocity_xy_toward 건).
비전이 끊기면 명령을 CMD_HOLD_S 동안 유지 후 CMD_DECAY_S에 걸쳐 0으로 선형 감쇠.
FRD->NED 속도 변환은 발행 시점의 현재 yaw(heading)만 사용한다.
접근 속도 상한은 V_MAX(=PX4 MPC_XY_VEL_MAX 0.3, sim/gcs_keepalive.py)와 정합.
오프보드 setpoint 스트림(on_setpoint_timer)은 상태 판단 로직(on_timer)과 분리된
별도 타이머 + MultiThreadedExecutor로 돌려서, 로직 쪽이 느려져도 PX4가 요구하는
안정적인 스트림(>2Hz)은 항상 유지한다.

안전 브링업 단계는 bringup_level 파라미터로 순서대로 검증한다:
  0 LOG_ONLY     : 오프보드 미발행/미arm. 변환된 목표 NED만 로그 (변환 검증).
  1 HOVER_HOLD   : arm+offboard, 제자리 유지만 (탐색/접근 비활성).
  2 YAW          : + 탐색 회전, 접근 시 yaw축만 서보 (수평 속도 0).
  3 YAW_LATERAL  : + lateral축도 서보 (forward는 고정, standoff 접근 없음).
  4 FULL         : forward 포함 전체 (표준 접근 시퀀스).

레벨과 무관하게 LOG_ONLY는 항상 레벨 4 기준으로 계산만 하고 로그만 남긴다
(변환 로직 자체를 검증하기 위함)."""
import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleAttitude,
)
from geometry_msgs.msg import PoseStamped

from kiosk_vision.wall_geometry import WALL_ID_BASE, quat_to_rotmat, yaw_err_from_R, wrap_deg_diff

LOG_ONLY, HOVER_HOLD, YAW, YAW_LATERAL, FULL = range(5)
LEVEL_NAMES = ['LOG_ONLY', 'HOVER_HOLD', 'YAW', 'YAW_LATERAL', 'FULL']

CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
# 오프보드 setpoint 스트림 주기. PX4는 오프보드 유지에 >2Hz를 요구하는데, 상태 판단
# 로직(on_timer)은 비전 처리 부하로 그 아래까지 느려진 적이 있어 스트림만 떼어낸다.
SETPOINT_HZ = 20.0
SETPOINT_DT = 1.0 / SETPOINT_HZ
ARM_TICK = 10  # 예제(offboard_control.py)와 동일하게 setpoint 10회 스트리밍 후 arm

# 카메라 마운트고 vs 마커 중심고 (docs/PROGRESS.md 2026-09-22 '카메라 마운트고 정렬' 참고).
# takeoff_alt 기본값을 이 둘로부터 유도한다 — 비행 고도를 마커 중심고와 그대로 맞추면
# 카메라가 mount_offset만큼 위에서 내려다보게 되어, standoff 근접 시 마커 패턴 아래쪽
# 행이 화각 밖으로 잘려나간다(실측: 1.0m 거리에서 3마커 게이트 통과율 0%).
MARKER_CENTER_HEIGHT = 1.5  # m — 벽 마커 패턴 중심 높이 (sim/worlds/kiosk_4walls.sdf 벽 pose z)
CAMERA_MOUNT_OFFSET = 0.242  # m — base_link 위 카메라 장착고
# (PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf CameraJoint pose z)

# ---- IBVS 속도 서보 튜닝 상수 (같은 이름의 소문자 ROS 파라미터로 오버라이드 가능) ----
# 게인은 비전 ~2Hz + 지연을 감안해 보수적으로(시정수 수 초). 시뮬 실측 접근속도 ~0.19m/s 기준.
KP_FWD = 0.25  # 1/s — 전진 오차[m] -> 전진 속도[m/s]
KP_LAT = 0.3   # 1/s — 좌우 오프셋[m] -> 우측 속도[m/s]
KP_YAW = 0.5   # 1/s — yaw 오차[rad] -> yaw rate[rad/s]
V_MAX = 0.3    # m/s — 축별 클립. PX4 MPC_XY_VEL_MAX(0.3)보다 크게 잡아봐야 PX4가 클립함
YAW_RATE_MAX_DEG = 20.0  # deg/s
# 단계 게이팅: yaw가 이만큼 틀어져 있으면 전진 0, 측면 감쇠 (틀어진 채 벽으로 돌진 방지)
YAW_GATE_DEG = 10.0
YAW_GATE_LAT_SCALE = 0.3
# 비전 명령 유실 안전: 정상 프레임 간격보다 길게 유지 후 선형 감쇠 -> 0.
# 0.6 -> 0.8: SITL 실측 PnP 간격 중앙 0.40s지만 p90 0.80s, 0.6s 초과가 30%라 정상
# 간격에서도 명령이 깎였음. 최대 공백 2.4s여도 0.8+0.6=1.4s 안에 정지.
CMD_HOLD_S = 0.8
CMD_DECAY_S = 0.6
# HOLD 히스테리시스: 진입은 tol_*(타이트), 이탈은 아래(느슨) — 데드존 안에선 v=0.
TOL_FORWARD_EXIT = 0.12  # m
# 좌우 0.20 -> 0.12: SITL에서 HOLD 드리프트로 lat≈0.16m(fwd≈0.96m)일 때 마커 2개만 보여 pose가
# 3s 끊김(REACQUIRE). 관측 실패점 0.16 아래 0.04 마진. 드리프트 ~2mm/s라 재보정은 수십 초에 한 번.
TOL_LATERAL_EXIT = 0.12  # m
TOL_YAW_EXIT_DEG = 6.0
# 근거리 유실 후퇴 복구: standoff 근처에서 놓친 건 대부분 상하 화각 잘림(타겟은 정면에
# 그대로) — 여기서 yaw 스윕은 마커를 더 잃게 만든다(SITL run3: 1mk까지, 공백 13.6s).
# 마지막 전진거리가 RETREAT_NEAR_FWD_M 미만이면 grace/스윕 대신 뒤로 물러나 화각을 되찾는다.
# 절대위치가 없으므로 거리는 시간으로 제한(0.1m/s x 3s = 0.3m) — 기체 뒤 여유 공간 전제.
RETREAT_NEAR_FWD_M = 1.3  # m
RETREAT_SPEED = 0.1       # m/s
RETREAT_MAX_S = 3.0       # s


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clip(v, lim):
    return max(-lim, min(lim, v))


class ApproachControlNode(Node):
    def __init__(self):
        super().__init__('approach_control_node')
        self.declare_parameter('bringup_level', LOG_ONLY)
        self.declare_parameter('pose_topic', '/target/pose')
        # aruco_pnp_node의 target_wall과 같은 값을 줘야 한다. 이 벽(frame_id='wall_<벽>')이
        # 아닌 pose는 버린다 — 이전 실행에서 고아로 남은 aruco_pnp_node(다른 target_wall)가
        # 같은 토픽에 발행하면 엉뚱한 벽으로 접근하던 문제 방어 (docs/PROGRESS.md 참고).
        self.declare_parameter('target_wall', '동')  # 북/동/남/서
        # 0.6m -> 1.0m: 1m 벽 패널 기준으로도 접근 끝(정지 지점)에서 4마커가 화각에
        # 남아 aruco_pnp_node의 마커 수 게이트(REQUIRED_MARKERS=4)가 막판에 끊기지
        # 않도록 여유를 둠.
        # 1.0m -> 1.15m (SCRUM-22): 속도 모드 HOLD 드리프트(피치 2~3°, 고도 1~2cm) 중 마커가
        # 잘리는 지배 축은 상하. IMX214 VFOV 54.5°, 패턴 반높이 0.45m 기준 상하 여유가
        # 1.0m에서 0.065m(0.96m면 0.045m)뿐이라 다 소진됐음 -> 1.15m면 ~0.14m.
        # 이 값은 제어(서보잉) 검증용 — 이 0.9m 패턴으로는 50cm 미션 거리 자체가 화각 밖.
        self.declare_parameter('standoff', 1.15)
        self.declare_parameter('search_yaw_rate_deg', 20.0)
        self.declare_parameter('kp_fwd', KP_FWD)
        self.declare_parameter('kp_lat', KP_LAT)
        self.declare_parameter('kp_yaw', KP_YAW)
        self.declare_parameter('v_max', V_MAX)
        self.declare_parameter('yaw_rate_max_deg', YAW_RATE_MAX_DEG)
        self.declare_parameter('cmd_hold_s', CMD_HOLD_S)
        self.declare_parameter('cmd_decay_s', CMD_DECAY_S)
        # solvePnP 거울해(flip) 사이 chatter 완화용 EMA 저역통과.
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('tol_forward', 0.06)
        # 6cm -> 12cm: approach_align 실측 lateral 노이즈 바닥이 약 3~12cm라 6cm는
        # 노이즈보다 빡빡해서 HOLD<->APPROACH가 계속 토글됐음 (docs/PROGRESS.md).
        # 12cm -> 8cm (SCRUM-22): 이탈 임계(0.12)를 마커 가시 범위에 맞춰 조이면서 히스테리시스
        # 0.04 유지. 당시 토글 문제는 진입=이탈 단일 임계 때문이었고 지금은 이탈이 따로 있음.
        self.declare_parameter('tol_lateral', 0.08)
        self.declare_parameter('tol_yaw_deg', 3.0)  # 실측 yaw 진동폭(~±2deg)이 이미 여유 있어 유지
        # HOLD 이탈 임계(진입 tol_*보다 느슨) — 속도 모드엔 위치 락이 없어 HOLD 중 느린
        # 드리프트가 생기는데, 진입=이탈 임계면 tol_forward 0.06 경계에서 chatter가 남.
        self.declare_parameter('tol_forward_exit', TOL_FORWARD_EXIT)
        self.declare_parameter('tol_lateral_exit', TOL_LATERAL_EXIT)
        self.declare_parameter('tol_yaw_exit_deg', TOL_YAW_EXIT_DEG)
        # 실기 마운트나 마커 높이가 바뀌면 두 파라미터를 오버라이드 — takeoff_alt 기본값이
        # 따라간다(바로 아래).
        self.declare_parameter('marker_center_height', MARKER_CENTER_HEIGHT)
        self.declare_parameter('camera_mount_offset', CAMERA_MOUNT_OFFSET)
        default_takeoff_alt = (float(self.get_parameter('marker_center_height').value)
                                - float(self.get_parameter('camera_mount_offset').value))
        # 카메라 광축을 마커 중심고에 맞춰 근접 시 전체 마커 가시성 확보.
        self.declare_parameter('takeoff_alt', default_takeoff_alt)  # m (NED z = -takeoff_alt)
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
        self.declare_parameter('retreat_near_fwd_m', RETREAT_NEAR_FWD_M)
        self.declare_parameter('retreat_speed', RETREAT_SPEED)
        self.declare_parameter('retreat_max_s', RETREAT_MAX_S)

        self.level = int(self.get_parameter('bringup_level').value)
        self.target_wall = self.get_parameter('target_wall').value
        if self.target_wall not in WALL_ID_BASE:
            raise ValueError(f"target_wall='{self.target_wall}' invalid, must be one of {list(WALL_ID_BASE)}")
        self.expected_frame_id = f'wall_{self.target_wall}'
        self.standoff = float(self.get_parameter('standoff').value)
        self.search_yaw_rate = math.radians(float(self.get_parameter('search_yaw_rate_deg').value))
        self.kp_fwd = float(self.get_parameter('kp_fwd').value)
        self.kp_lat = float(self.get_parameter('kp_lat').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.yaw_rate_max = math.radians(float(self.get_parameter('yaw_rate_max_deg').value))
        self.cmd_hold_s = float(self.get_parameter('cmd_hold_s').value)
        self.cmd_decay_s = float(self.get_parameter('cmd_decay_s').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.tol_forward = float(self.get_parameter('tol_forward').value)
        self.tol_lateral = float(self.get_parameter('tol_lateral').value)
        self.tol_yaw = math.radians(float(self.get_parameter('tol_yaw_deg').value))
        self.tol_forward_exit = float(self.get_parameter('tol_forward_exit').value)
        self.tol_lateral_exit = float(self.get_parameter('tol_lateral_exit').value)
        self.tol_yaw_exit = math.radians(float(self.get_parameter('tol_yaw_exit_deg').value))
        self.marker_center_height = float(self.get_parameter('marker_center_height').value)
        self.camera_mount_offset = float(self.get_parameter('camera_mount_offset').value)
        self.takeoff_alt = float(self.get_parameter('takeoff_alt').value)
        self.target_lost_timeout = float(self.get_parameter('target_lost_timeout').value)
        self.reacquire_grace_s = float(self.get_parameter('reacquire_grace_s').value)
        self.reacquire_sweep = math.radians(float(self.get_parameter('reacquire_sweep_deg').value))
        self.reacquire_sweep_rate = math.radians(float(self.get_parameter('reacquire_sweep_rate_deg').value))
        self.reacquire_sweep_max_s = float(self.get_parameter('reacquire_sweep_max_s').value)
        self.retreat_near_fwd = float(self.get_parameter('retreat_near_fwd_m').value)
        self.retreat_speed = float(self.get_parameter('retreat_speed').value)
        self.retreat_max_s = float(self.get_parameter('retreat_max_s').value)

        # 콜백 그룹 3개 + MultiThreadedExecutor(main() 참고):
        # - logic_group(상호배제): 비전 콜백(EMA·decide)과 on_timer(상태머신). 둘 다 같은
        #   상태(EMA, state, _reacquire_*)를 읽고 쓰므로 직렬화한다. 재진입 그룹이면 on_timer가
        #   50ms를 넘길 때 자기 자신과 겹치거나 비전 콜백 두 개가 EMA를 동시에 갱신할 수 있었다.
        # - stream_group(상호배제): on_setpoint_timer(PX4 오프보드 스트림) 전용 — 로직 쪽이
        #   느려져도 스트림은 굶지 않는다(2-시계 독립성 유지).
        # - sensor_group(재진입): 자세/로컬위치 구독. 참조 대입만 하므로 겹쳐도 무해하고,
        #   로직 쪽에 묶이면 발행부가 쓰는 yaw가 묵을 수 있어 따로 둔다.
        logic_group = MutuallyExclusiveCallbackGroup()
        stream_group = MutuallyExclusiveCallbackGroup()
        sensor_group = ReentrantCallbackGroup()

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
                                  self.on_local_position, px4_qos, callback_group=sensor_group)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                  self.on_attitude, px4_qos, callback_group=sensor_group)

        pose_topic = self.get_parameter('pose_topic').value
        self.pose_topic = pose_topic
        self.create_subscription(PoseStamped, pose_topic, self.on_target_pose, 10, callback_group=logic_group)

        self.vlp = None
        self.att = None
        self.last_target_pose = None
        self.last_target_time = None
        self.ema_lateral = None
        self.ema_yaw_err_deg = None

        # decide()가 비전 프레임마다 한 번 계산하는 상대속도 명령 (그 사이엔 그대로 유지).
        # (vf, vr, yaw_rate, stamp, forward_err, lateral_err, yaw_err)를 한 튜플로 묶어
        # 통째로 교체한다 — 비전 콜백과 on_timer가 다른 스레드에서 도니까, 필드를 따로
        # 두면 명령은 새 프레임 건데 오차는 이전 프레임 값인 조합으로 HOLD 판정이 날 수 있다.
        self.vision_cmd = None
        # on_timer가 상태별로 고르고 on_setpoint_timer가 읽는 최신 기체(FRD) 명령
        # (vf, vr, yaw_rate, stamp). stamp는 명령의 출처 시각 — APPROACH면 비전 프레임
        # 시각이라 비전이 끊기면 발행부에서 hold->decay가 걸린다.
        self.cmd = None

        self.state = 'SEARCH'
        self.offboard_setpoint_counter = 0

        self._reacquire_since = None
        self._reacquire_center_yaw = None
        self._reacquire_sweep_dir = 1.0
        self._reacquire_retreat = False
        self._last_tick_time = None

        self.create_timer(DT, self.on_timer, callback_group=logic_group)
        self.create_timer(SETPOINT_DT, self.on_setpoint_timer, callback_group=stream_group)
        self.get_logger().info(
            f'approach_control_node start target_wall={self.target_wall} '
            f'bringup_level={self.level}({LEVEL_NAMES[self.level]}) '
            f'standoff={self.standoff}m takeoff_alt={self.takeoff_alt}m '
            f'(marker_center_height={self.marker_center_height}m - camera_mount_offset={self.camera_mount_offset}m) '
            f'kp_fwd={self.kp_fwd} kp_lat={self.kp_lat} kp_yaw={self.kp_yaw} v_max={self.v_max} '
            f'yaw_rate_max={math.degrees(self.yaw_rate_max):.0f}deg/s '
            f'cmd_hold={self.cmd_hold_s}s cmd_decay={self.cmd_decay_s}s ema_alpha={self.ema_alpha}')

    # ---- subscriptions ----
    def on_local_position(self, msg):
        # 비교 로그 전용 — 제어 경로는 수평 절대위치를 쓰지 않으므로 xy_valid(실내 GPS 없음
        # 이면 False)를 게이트로 걸지 않는다.
        self.vlp = msg

    def on_attitude(self, msg):
        self.att = msg

    def on_target_pose(self, msg):
        if msg.header.frame_id != self.expected_frame_id:
            n_pub = self.count_publishers(self.pose_topic)
            self.get_logger().warn(
                f"[{self.pose_topic}] frame_id='{msg.header.frame_id}' != '{self.expected_frame_id}' -> 버림 "
                f"(publisher {n_pub}개 — 다른 target_wall의 aruco_pnp_node가 살아있는지 확인)",
                throttle_duration_sec=2.0)
            return
        self.last_target_pose = msg
        self.last_target_time = self.get_clock().now()

        raw_lateral = msg.pose.position.x
        raw_forward = msg.pose.position.z
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

        self.decide(raw_forward)

    def decide(self, forward):
        """비전 프레임 도착 시 상대오차만으로 기체(FRD) 속도 + yaw rate를 계산해
        self.vision_cmd에 저장만 한다(발행은 on_setpoint_timer 전담). 절대위치 미사용.
        부호는 기존 앵커 식(target_yaw = yaw + gain*yaw_err, lateral +=FRD right)과 동일
        — SCRUM-25에서 폐루프로 검증된 rel 부호 그대로."""
        forward_err = forward - self.standoff
        lateral_err = self.ema_lateral
        yaw_err = math.radians(self.ema_yaw_err_deg)

        vf = clip(self.kp_fwd * forward_err, self.v_max)
        vr = clip(self.kp_lat * lateral_err, self.v_max)
        yaw_rate = clip(self.kp_yaw * yaw_err, self.yaw_rate_max)

        # 단계 게이팅: yaw -> lateral -> forward. 틀어진 채로는 전진하지 않는다.
        if abs(yaw_err) > math.radians(YAW_GATE_DEG):
            vf = 0.0
            vr *= YAW_GATE_LAT_SCALE

        gate_level = FULL if self.level == LOG_ONLY else self.level
        if gate_level < FULL:
            vf = 0.0
        if gate_level < YAW_LATERAL:
            vr = 0.0

        self.vision_cmd = (vf, vr, yaw_rate, self.get_clock().now(),
                           forward_err, lateral_err, yaw_err)

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

    # ---- main loop ----
    def on_timer(self):
        if self.att is None:
            return

        now = self.get_clock().now()
        # 콜백이 이론상 20Hz(DT)지만 WSL에서 시스템 부하로 실제 간격이 늘어날 수 있음
        # (실측: gz 렌더링/비전 처리로 인해 몇 배까지 지연됨, docs/PROGRESS.md 참고).
        # 회전은 yawspeed로 커맨드하므로 적분엔 안 쓰고, 루프 건강 확인용으로 로그에 남긴다.
        if self._last_tick_time is None:
            dt = DT
        else:
            dt = max((now - self._last_tick_time).nanoseconds * 1e-9, 1e-3)
        self._last_tick_time = now

        if self.level == HOVER_HOLD:
            self.state = 'SEARCH'
            cmd = (0.0, 0.0, 0.0, now)
        else:
            cmd = self.run_state_machine(now)

        vf, vr, yaw_rate, _ = cmd
        vc = self.vision_cmd
        err_str = (f'fwd_err={vc[4]:+.3f} lat_err={vc[5]:+.3f} yaw_err={math.degrees(vc[6]):+.1f}deg '
                   if vc is not None else '')
        # cur_ned는 비교용 로그 전용 — 제어 경로(decide/상태머신/발행부)는 참조하지 않는다.
        ned_str = (f'cur_ned(log)=({self.vlp.x:.3f},{self.vlp.y:.3f},{self.vlp.z:.3f}) '
                   if self.vlp is not None else '')
        log = (f'[{LEVEL_NAMES[self.level]}][{self.state}] {err_str}'
               f'cmd_frd=(vf={vf:+.3f},vr={vr:+.3f},yaw_rate={math.degrees(yaw_rate):+.1f}deg/s) '
               f'yaw={math.degrees(self.current_yaw()):+.1f}deg {ned_str}dt={dt:.3f}s')

        if self.level == LOG_ONLY:
            self.get_logger().info(log, throttle_duration_sec=0.5)
            return  # 오프보드 발행/arm 없음 — 드론은 움직이지 않는다

        # 실제 발행은 on_setpoint_timer가 전담한다 — 여기서는 명령 스냅샷만 통째로 교체
        # (튜플 하나를 바꾸므로 스트림 쪽이 반쯤 갱신된 명령을 읽는 일이 없다).
        self.cmd = cmd
        self.get_logger().info(log, throttle_duration_sec=0.5)

    def cmd_scale(self, stamp):
        """명령 출처 시각 기준 유실 안전 감쇠: cmd_hold_s까지 1, 이후 cmd_decay_s에 걸쳐 0."""
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age <= self.cmd_hold_s:
            return 1.0
        return max(0.0, 1.0 - (age - self.cmd_hold_s) / self.cmd_decay_s)

    def on_setpoint_timer(self):
        """오프보드 스트림 전담(on_timer와 별도 타이머·스레드). 최신 기체(FRD) 명령을
        현재 yaw로만 NED 속도로 돌려 20Hz로 재발행한다 — 비전(≈2Hz)이 명령을 갱신하는
        주기와 PX4로 나가는 주기를 분리해 둔다."""
        if self.level == LOG_ONLY:
            return
        cmd = self.cmd
        if cmd is None or self.att is None:
            return  # 아직 자세 미수신 -> 발행할 명령 없음

        vf, vr, yaw_rate, stamp = cmd
        s = self.cmd_scale(stamp)
        vf, vr, yaw_rate = vf * s, vr * s, yaw_rate * s
        yaw = self.current_yaw()
        vx = vf * math.cos(yaw) - vr * math.sin(yaw)
        vy = vf * math.sin(yaw) + vr * math.cos(yaw)

        self.publish_offboard_heartbeat()
        self.publish_trajectory_setpoint(vx, vy, yaw_rate)

        if self.offboard_setpoint_counter == ARM_TICK:
            self.engage_offboard_mode()
            self.arm()
        if self.offboard_setpoint_counter <= ARM_TICK:
            self.offboard_setpoint_counter += 1

    def _handle_lost(self, now):
        """타겟 유실 시 곧바로 전체 SEARCH로 튀지 않고 REACQUIRE 3단계를 거친다:
        ①grace(제자리 대기) — 단 standoff 근처 유실이면 대신 후퇴(RETREAT_*)
        ②마지막 방향 근방 좁은 스윕 ③그래도 못 찾으면 전체 SEARCH.
        제자리 = 수평 속도 0, 회전 = yawspeed (절대위치 없음)."""
        cur_yaw = self.current_yaw()

        if self.state in ('APPROACH', 'HOLD'):
            self.state = 'REACQUIRE'
            self._reacquire_since = now
            self._reacquire_center_yaw = cur_yaw
            self._reacquire_sweep_dir = 1.0
            vc = self.vision_cmd
            last_fwd = vc[4] + self.standoff if vc is not None else math.inf
            gate_level = FULL if self.level == LOG_ONLY else self.level
            self._reacquire_retreat = gate_level >= FULL and last_fwd < self.retreat_near_fwd
            if self._reacquire_retreat:
                self.get_logger().info(
                    f'타겟 유실(근거리 fwd={last_fwd:.2f}m) -> REACQUIRE(후퇴 '
                    f'{self.retreat_speed}m/s x {self.retreat_max_s}s) 진입')
            else:
                self.get_logger().info('타겟 유실 -> REACQUIRE(제자리 대기) 진입')

        if self.state == 'REACQUIRE':
            elapsed = (now - self._reacquire_since).nanoseconds * 1e-9
            first_phase_s = self.retreat_max_s if self._reacquire_retreat else self.reacquire_grace_s
            if elapsed <= first_phase_s:
                if self._reacquire_retreat:
                    # ① 근거리: 뒤로 물러나 상하 화각 여유를 되찾는다 (yaw 유지)
                    return (-self.retreat_speed, 0.0, 0.0, now)
                return (0.0, 0.0, 0.0, now)  # ① grace: 제자리 대기

            # 후퇴로도 못 찾았으면 단순 화각 잘림이 아니므로 기존 스윕 -> SEARCH로 넘어간다.
            if elapsed <= first_phase_s + self.reacquire_sweep_max_s:
                # ② 마지막 방향 기준 좁은 스윕 (느린 속도로 왕복) — 실제 yaw가 경계를
                # 넘으면 방향만 뒤집는다.
                offset = wrap_pi(cur_yaw - self._reacquire_center_yaw)
                if offset > self.reacquire_sweep and self._reacquire_sweep_dir > 0:
                    self._reacquire_sweep_dir = -1.0
                elif offset < -self.reacquire_sweep and self._reacquire_sweep_dir < 0:
                    self._reacquire_sweep_dir = 1.0
                return (0.0, 0.0, self.reacquire_sweep_rate * self._reacquire_sweep_dir, now)

            # ③ 좁은 스윕도 실패 -> 전체 SEARCH로 승격
            self.get_logger().info('REACQUIRE 실패(좁은 스윕 시간초과) -> 전체 SEARCH 회전으로 전환')
            self.state = 'SEARCH'

        return (0.0, 0.0, self.search_yaw_rate, now)

    def run_state_machine(self, now):
        vc = self.vision_cmd  # 비전 콜백이 다른 스레드에서 갈아끼우므로 한 번만 읽어 쓴다
        if self.target_lost() or vc is None:
            return self._handle_lost(now)

        vf, vr, yaw_rate, stamp, forward_err, lateral_err, yaw_err = vc
        # HOLD 히스테리시스: 진입은 tol_*(타이트), 이미 HOLD면 *_exit(느슨)를 벗어날
        # 때까지 유지 — 데드존 안에선 v=0, 벗어나면 APPROACH로 재서보.
        if self.state == 'HOLD':
            tol_f, tol_l, tol_y = self.tol_forward_exit, self.tol_lateral_exit, self.tol_yaw_exit
        else:
            tol_f, tol_l, tol_y = self.tol_forward, self.tol_lateral, self.tol_yaw
        gate_level = FULL if self.level == LOG_ONLY else self.level
        checks = [abs(yaw_err) < tol_y]
        if gate_level >= YAW_LATERAL:
            checks.append(abs(lateral_err) < tol_l)
        if gate_level >= FULL:
            checks.append(abs(forward_err) < tol_f)

        if all(checks):
            self.state = 'HOLD'
            return (0.0, 0.0, 0.0, now)
        self.state = 'APPROACH'
        return (vf, vr, yaw_rate, stamp)

    # ---- px4 command helpers (offboard_control.py 예제와 동일 패턴) ----
    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        # velocity 모드: PX4가 수평 위치 제어를 끄고(고도 제어는 유지) offboard 진입 조건도
        # local_position이 아니라 local_velocity 유효성만 본다
        # (PX4 commander ModeUtil/control_mode.cpp, offboardCheck.cpp).
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, yaw_rate):
        """혼합 setpoint: 수평은 NED 속도, 고도만 position(z=-takeoff_alt, 이륙도 이걸로),
        yaw는 NaN + yawspeed. 수평 position이 NaN이라 PX4가 수평 위치를 붙잡지 않는다."""
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, float(-self.takeoff_alt)]
        msg.velocity = [float(vx), float(vy), math.nan]
        msg.yaw = math.nan
        msg.yawspeed = float(yaw_rate)
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
    # 로직 타이머와 setpoint 스트림 타이머가 서로를 굶기지 않도록 멀티스레드로 돈다.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
