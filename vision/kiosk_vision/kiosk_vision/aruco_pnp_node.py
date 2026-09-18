#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위치계산부 + 판단부: 검출된 마커를 벽별(ID 그룹)로 묶어 각 벽을 통합
solvePnP(IPPE)로 풀고, target_wall 파라미터가 지정한 벽이 보이면 그 벽의
pose만 /target/pose 로 발행한다 (판단부: 타겟 벽이 안 보이면 계속 탐색)."""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from kiosk_vision.wall_geometry import (
    MARKER_SIZE, CORNER_LAYOUT, WALL_ID_BASE,
    marker_id_to_wall, marker_corners_3d, rotmat_to_quat, yaw_err_from_R,
)

ARUCO_DICT = cv2.aruco.DICT_4X4_50
REPROJ_ERR_WARN = 2.0


class ArucoPnPNode(Node):
    def __init__(self):
        super().__init__('aruco_pnp_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('pose_topic', '/target/pose')
        self.declare_parameter('visible_topic', '/target/visible')
        self.declare_parameter('target_wall', '동')  # 북/동/남/서
        img_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        visible_topic = self.get_parameter('visible_topic').value
        self.target_wall = self.get_parameter('target_wall').value
        if self.target_wall not in WALL_ID_BASE:
            raise ValueError(f"target_wall='{self.target_wall}' invalid, must be one of {list(WALL_ID_BASE)}")

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        else:
            self.detector = None
            self.aruco_params = cv2.aruco.DetectorParameters_create()  # OpenCV 4.6 폴백

        self.create_subscription(CameraInfo, info_topic, self.on_camera_info, 10)
        self.create_subscription(Image, img_topic, self.on_image, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.visible_pub = self.create_publisher(Bool, visible_topic, 10)
        self.latest_pose = None
        self.get_logger().info(
            f'aruco_pnp_node start img={img_topic} info={info_topic} target_wall={self.target_wall}')

    def on_camera_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        if self.D.size == 0:
            self.D = np.zeros((5,), dtype=np.float64)

    def _detect(self, gray):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        return corners, ids

    def _group_by_wall(self, ids, corners):
        """검출 마커를 ID로 벽별 그룹핑 -> {wall_name: (obj_pts, img_pts)}"""
        groups = {}
        for mid, cn in zip(ids, corners):
            mid = int(mid)
            wall_name, corner_off = marker_id_to_wall(mid)
            if wall_name is None:
                continue
            cx, cy = CORNER_LAYOUT[corner_off]
            obj_pts, img_pts = groups.setdefault(wall_name, ([], []))
            obj_pts += marker_corners_3d(cx, cy, MARKER_SIZE)
            img_pts += list(cn.reshape(4, 2))
        return groups

    def _solve_wall(self, wall_name, obj_pts, img_pts):
        if len(obj_pts) < 4:
            return None
        obj_pts = np.array(obj_pts, dtype=np.float32)
        img_pts = np.array(img_pts, dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            self.get_logger().warn(f'[{wall_name}] solvePnP failed')
            return None
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.K, self.D)
        reproj_err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)))
        tv = tvec.flatten()
        R, _ = cv2.Rodrigues(rvec)
        yaw_err = yaw_err_from_R(R)
        return dict(
            wall=wall_name, forward=float(tv[2]), lateral=float(tv[0]), vertical=float(tv[1]),
            dist=float(np.linalg.norm(tv)), yaw_err=yaw_err, reproj_err=reproj_err,
            n_markers=len(obj_pts) // 4, R=R, tv=tv,
        )

    def on_image(self, msg):
        if self.K is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)
        if ids is None:
            self.visible_pub.publish(Bool(data=False))
            return
        ids = ids.flatten()

        groups = self._group_by_wall(ids, corners)
        results = {}
        for wall_name, (obj_pts, img_pts) in groups.items():
            r = self._solve_wall(wall_name, obj_pts, img_pts)
            if r is not None:
                results[wall_name] = r

        if results:
            seen = ', '.join(f"{w}({r['n_markers']}mk,{r['reproj_err']:.1f}px)" for w, r in results.items())
            self.get_logger().info(f'[walls seen] {seen}')

        target = results.get(self.target_wall)
        self.visible_pub.publish(Bool(data=bool(target)))
        if target is None:
            return

        self.latest_pose = target
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = f'wall_{self.target_wall}'
        ps.pose.position.x = target['lateral']
        ps.pose.position.y = target['vertical']
        ps.pose.position.z = target['forward']
        qx, qy, qz, qw = rotmat_to_quat(target['R'])
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        lvl = self.get_logger().warn if target['reproj_err'] > REPROJ_ERR_WARN else self.get_logger().info
        lvl(f"[PnP target={self.target_wall}] {target['n_markers']}mk | fwd {target['forward']:.3f}m "
            f"| lat {target['lateral']:+.3f} | yaw {target['yaw_err']:+.1f}deg | reproj {target['reproj_err']:.2f}px")

    def get_pose(self):
        return self.latest_pose


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPnPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
