#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

# ===== 설정 (시뮬 켠 뒤 실제값으로 교체) =====
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE = 0.150   # 마커 한 변 실제 길이 [m] (sim/generate_marker.py MARKER_MM=150)
MARKER_LAYOUT = {     # 원점=4마커 중앙, X오른쪽/Y위/Z=0 [m], key=ArUco id
    0: (-0.375,  0.375), 1: ( 0.375,  0.375),   # top-left, top-right
    2: ( 0.375, -0.375), 3: (-0.375, -0.375),   # bottom-right, bottom-left
}
REPROJ_ERR_WARN = 2.0

def marker_corners_3d(cx, cy, s):
    h = s / 2.0
    return [(cx-h, cy+h, 0.0), (cx+h, cy+h, 0.0),
            (cx+h, cy-h, 0.0), (cx-h, cy-h, 0.0)]  # TL,TR,BR,BL

def rotmat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t+1.0)*2; w=0.25*s
        x=(R[2,1]-R[1,2])/s; y=(R[0,2]-R[2,0])/s; z=(R[1,0]-R[0,1])/s
    else:
        i = np.argmax([R[0,0],R[1,1],R[2,2]])
        if i==0:
            s=np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2
            w=(R[2,1]-R[1,2])/s; x=0.25*s; y=(R[0,1]+R[1,0])/s; z=(R[0,2]+R[2,0])/s
        elif i==1:
            s=np.sqrt(1.0-R[0,0]+R[1,1]-R[2,2])*2
            w=(R[0,2]-R[2,0])/s; x=(R[0,1]+R[1,0])/s; y=0.25*s; z=(R[1,2]+R[2,1])/s
        else:
            s=np.sqrt(1.0-R[0,0]-R[1,1]+R[2,2])*2
            w=(R[1,0]-R[0,1])/s; x=(R[0,2]+R[2,0])/s; y=(R[1,2]+R[2,1])/s; z=0.25*s
    return (x,y,z,w)

class ArucoPnPNode(Node):
    def __init__(self):
        super().__init__('aruco_pnp_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('pose_topic', '/target/pose')
        img_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.bridge = CvBridge(); self.K = None; self.D = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        else:
            self.detector = None
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.create_subscription(CameraInfo, info_topic, self.on_camera_info, 10)
        self.create_subscription(Image, img_topic, self.on_image, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.latest_pose = None
        self.get_logger().info(f'aruco_pnp_node start img={img_topic} info={info_topic}')

    def on_camera_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D = np.array(msg.d, dtype=np.float64)
        if self.D.size == 0: self.D = np.zeros((5,), dtype=np.float64)

    def on_image(self, msg):
        if self.K is None: return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None: return
        ids = ids.flatten()
        obj_pts, img_pts = [], []
        for mid, cn in zip(ids, corners):
            mid = int(mid)
            if mid not in MARKER_LAYOUT: continue
            cx, cy = MARKER_LAYOUT[mid]
            obj_pts += marker_corners_3d(cx, cy, MARKER_SIZE)
            img_pts += list(cn.reshape(4,2))
        if len(obj_pts) < 4: return
        obj_pts = np.array(obj_pts, dtype=np.float32)
        img_pts = np.array(img_pts, dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            self.get_logger().warn('solvePnP failed'); return
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.K, self.D)
        reproj_err = float(np.mean(np.linalg.norm(proj.reshape(-1,2)-img_pts, axis=1)))
        tv = tvec.flatten()
        forward=float(tv[2]); lateral=float(tv[0]); vertical=float(tv[1])
        dist_line=float(np.linalg.norm(tv))
        R, _ = cv2.Rodrigues(rvec)
        normal = R @ np.array([0.0,0.0,1.0])
        yaw_err = float(np.degrees(np.arctan2(normal[0], -normal[2])))
        self.latest_pose = dict(forward=forward, lateral=lateral, vertical=vertical,
                                dist=dist_line, yaw_err=yaw_err, reproj_err=reproj_err,
                                n_markers=len(obj_pts)//4)
        ps = PoseStamped(); ps.header = msg.header
        ps.pose.position.x=float(tv[0]); ps.pose.position.y=float(tv[1]); ps.pose.position.z=float(tv[2])
        qx,qy,qz,qw = rotmat_to_quat(R)
        ps.pose.orientation.x=qx; ps.pose.orientation.y=qy; ps.pose.orientation.z=qz; ps.pose.orientation.w=qw
        self.pose_pub.publish(ps)
        lvl = self.get_logger().warn if reproj_err > REPROJ_ERR_WARN else self.get_logger().info
        lvl(f'[PnP] {len(obj_pts)//4}mk | dist {forward:.3f}m (line {dist_line:.3f}) '
            f'| lat {lateral:+.3f} | yaw {yaw_err:+.1f}deg | reproj {reproj_err:.2f}px')

    def get_pose(self): return self.latest_pose

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPnPNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
