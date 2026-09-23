#!/usr/bin/env python3
"""Steps 5-8 of the mono-multiview 이동 구간 plan, built on top of
mono_multiview_capture_and_match.py's proven capture+ORB+match+RANSAC
(steps 1-4):

  5. Essential Matrix -> relative pose (rotation + unit-scale translation)
  6. Triangulation -> 3D points, still in unit-baseline scale
  7. Scale correction -> PX4's own position estimate gives the real
     frame-A-to-frame-B displacement; recoverPose's translation is already
     unit-length, so that real displacement IS the scale factor.
  8. Time sync -> simplified here to "read telemetry immediately after each
     capture" (good enough for this two-shot test); the production ROS2 node
     will need a real image-timestamp <-> position-timestamp association
     (e.g. message_filters), not this shortcut.

Prints a final forward/lateral distance estimate (meters, camera-A frame) for
the matched points -- the same shape of output as aruco_pnp_node's PoseStamped,
so it can be cross-checked against PnP/PX4 ground truth.

TEMPORARY validation script, not the production node.
"""

import asyncio
import threading
import time

import cv2
import numpy as np
from gz.msgs10.camera_info_pb2 import CameraInfo
import gz.transport13 as gz_transport
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

from mono_multiview_capture_and_match import (
    ALT, APPROACH_NORTH, BASELINE_EAST, BASELINE_NORTH, SETTLE_SEC,
    capture_frame, detect_and_match, goto,
)

CAMERA_INFO_TOPIC = "/world/kiosk/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info"


def capture_camera_info():
    """Blocking single-message grab of camera intrinsics via gz-transport."""
    info = {}
    got = threading.Event()

    def cb(msg: CameraInfo):
        if got.is_set():
            return
        info["K"] = np.array(msg.intrinsics.k, dtype=np.float64).reshape(3, 3)
        got.set()

    node = gz_transport.Node()
    node.subscribe(CameraInfo, CAMERA_INFO_TOPIC, cb)
    if not got.wait(timeout=10):
        raise SystemExit("failed to get camera_info")
    return info["K"]


async def get_position_ned(drone):
    async for pv in drone.telemetry.position_velocity_ned():
        return pv.position


# Below this bounding-box diagonal, matched points are more likely clustered
# on a single ~0.15m ArUco marker (diagonal ~0.21m) than spread across the
# wall -- confirmed by inspecting mono_frame_a/b.png for a run whose plane-fit
# yaw was off by ~20deg from aruco_pnp ground truth: frame B (closer to the
# wall, narrower FOV coverage) only shared its top two markers with frame A,
# so only points from that shared region could match at all. Two markers
# ~0.75m apart give a diagonal well above this; a single marker's corners do
# not, so this threshold separates the two cases without needing to know how
# many markers were involved.
SPREAD_MIN_M = 0.4


def fit_plane(points):
    """Least-squares plane fit (SVD) to the scaled 3D points -- the kiosk wall
    is assumed planar. Returns (normal, centroid, spread). normal is oriented
    to face back toward the camera (negative Z), matching aruco_pnp_node's
    board-normal convention so the yaw formula below is directly comparable to
    its yaw_err. spread is the matched points' bounding-box diagonal [m]: the
    plane's normal (hence the yaw angle) is only well-constrained if the
    points span a meaningful area, not a single small cluster -- see
    SPREAD_MIN_M."""
    centroid = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - centroid)
    normal = vh[-1]
    if normal[2] > 0:
        normal = -normal
    spread = float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)))
    return normal, centroid, spread


def rotation_angle_deg(R):
    """Angle of the rotation R represents, via the trace formula. Sanity check
    only: we commanded yaw=0.0 throughout, so this should be close to 0 -- it
    is NOT the wall-relative angle (that comes from the plane normal), just a
    check that the drone held the commanded attitude between frame A and B."""
    cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_theta)))


def estimate_pose(pts_a, pts_b, K):
    """Steps 5-6: Essential Matrix -> pose -> triangulated points (unit scale)."""
    E, mask = cv2.findEssentialMat(pts_a, pts_b, K, method=cv2.RANSAC,
                                    prob=0.999, threshold=1.0)
    if E is None or E.shape != (3, 3):
        raise SystemExit(f"findEssentialMat failed (E shape: {None if E is None else E.shape})")

    inlier_a = pts_a[mask.ravel() == 1]
    inlier_b = pts_b[mask.ravel() == 1]
    print(f"Essential matrix inliers: {len(inlier_a)}/{len(pts_a)}")
    if len(inlier_a) < 5:
        raise SystemExit("too few essential-matrix inliers to recover pose")

    n_front, R, t, pose_mask = cv2.recoverPose(E, inlier_a, inlier_b, K)
    print(f"recoverPose: {n_front} points in front of both cameras "
          f"(t is unit-length: this is the mono scale-ambiguity step 7 fixes)")

    P0 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P1 = K @ np.hstack([R, t])
    pts4d = cv2.triangulatePoints(P0, P1, inlier_a.T, inlier_b.T)
    pts3d_unit = (pts4d[:3] / pts4d[3]).T  # Nx3, camera-A frame, unit baseline

    keep = pose_mask.ravel() > 0
    return R, t, pts3d_unit[keep]


def estimate_pose_homography(pts_a, pts_b, K):
    """Alternative to estimate_pose(), appropriate when the matched points lie
    on a single plane -- our kiosk wall, always, in this sim. Essential Matrix
    decomposition is known to be poorly conditioned for purely planar point
    sets (a real risk here, confirmed by the distance sweep: some pairs came
    back with a negative estimated distance -- a nonsensical result). Unlike
    estimate_pose(), this solves for the plane's normal directly from the 2D
    correspondences instead of triangulating noisy 3D points and fitting a
    plane to them afterward, so the wall-relative angle no longer depends on a
    second, separately-noisy estimation step.

    Returns (R, forward, lateral, vertical, yaw_deg, spread, inlier_count).
    forward/lateral/vertical/spread are scaled directly to meters here (unlike
    estimate_pose(), which returns unit-scale points for run() to scale) since
    the plane-relative point reconstruction needs the scale factor internally.
    """
    H, mask = cv2.findHomography(pts_a, pts_b, cv2.RANSAC, 3.0)
    if H is None:
        raise SystemExit("findHomography failed")
    inlier_count = int(mask.sum())
    print(f"Homography inliers: {inlier_count}/{len(pts_a)}")
    if inlier_count < 8:
        raise SystemExit("too few homography inliers")

    _, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, K)

    pts_a_norm = cv2.undistortPoints(pts_a.reshape(-1, 1, 2).astype(np.float32), K, None)
    pts_b_norm = cv2.undistortPoints(pts_b.reshape(-1, 1, 2).astype(np.float32), K, None)

    possible = cv2.filterHomographyDecompByVisibleRefpoints(
        Rs, Ns, pts_a_norm, pts_b_norm, pointsMask=mask)
    if possible is None or len(possible) == 0:
        raise SystemExit("no valid homography decomposition after filtering")
    idx = int(possible.ravel()[0])
    R = Rs[idx]
    t_internal = Ts[idx].ravel()   # translation / plane-depth (unscaled)
    n = Ns[idx].ravel()            # plane normal, camera-A frame, same internal scale

    inlier_a = pts_a[mask.ravel() == 1]
    return R, t_internal, n, inlier_a, inlier_count


def homography_to_metric(R, t_internal, n, inlier_a, K, real_baseline):
    """Scale the homography decomposition to meters using PX4's real baseline
    (step 7, same idea as estimate_pose()'s scale correction), then reconstruct
    each inlier point's 3D position by intersecting its camera ray with the
    known plane (n . X = 1 in the decomposition's internal units) rather than
    triangulating -- this leans on the planar assumption instead of fighting
    it, which is the whole point of using homography here."""
    scale = real_baseline / np.linalg.norm(t_internal)

    rays = cv2.undistortPoints(inlier_a.reshape(-1, 1, 2).astype(np.float64), K, None).reshape(-1, 2)
    rays = np.hstack([rays, np.ones((len(rays), 1))])  # Nx3, camera-A frame, unnormalized ray directions

    denom = rays @ n
    valid = np.abs(denom) > 1e-6
    s = np.full(len(rays), np.nan)
    s[valid] = 1.0 / denom[valid]
    pts3d_internal = s[:, None] * rays  # points on the plane, internal scale

    pts3d_m = pts3d_internal[valid] * scale

    n_facing = -n if n[2] > 0 else n
    forward = float(-np.dot(n_facing, pts3d_m.mean(axis=0)))
    centroid = pts3d_m.mean(axis=0)
    lateral, vertical = float(centroid[0]), float(centroid[1])
    yaw_deg = float(np.degrees(np.arctan2(n_facing[0], -n_facing[2])))
    spread = float(np.linalg.norm(pts3d_m.max(axis=0) - pts3d_m.min(axis=0)))
    return forward, lateral, vertical, yaw_deg, spread


async def run(K):
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Priming setpoint stream")
    for _ in range(10):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard start failed: {error._result.result}")
        await drone.action.disarm()
        return

    await goto(drone, 0.0, 0.0, 0.0, "Takeoff to 1.5m", SETTLE_SEC)
    await goto(drone, 0.0, 0.0, 0.0, "Rotate to yaw=0.0 (facing wall)", SETTLE_SEC)
    await goto(drone, APPROACH_NORTH, 0.0, 0.0, f"North {APPROACH_NORTH}m (approach)", SETTLE_SEC)

    loop = asyncio.get_running_loop()
    print("-- Capturing frame A + position")
    frame_a = await loop.run_in_executor(None, capture_frame, "a")
    pos_a = await get_position_ned(drone)
    t_a = time.time()
    print(f"-- frame A wall-clock time: {t_a:.3f}  north={pos_a.north_m:.3f}m")

    baseline_target_n = APPROACH_NORTH + BASELINE_NORTH
    baseline_target_e = BASELINE_EAST
    await goto(drone, baseline_target_n, baseline_target_e, 0.0,
               f"North {baseline_target_n}m East {baseline_target_e}m (baseline step)", SETTLE_SEC)

    print("-- Capturing frame B + position")
    frame_b = await loop.run_in_executor(None, capture_frame, "b")
    pos_b = await get_position_ned(drone)
    t_b = time.time()
    print(f"-- frame B wall-clock time: {t_b:.3f}  north={pos_b.north_m:.3f}m")

    print("-- Returning to origin")
    await goto(drone, 0.0, 0.0, 0.0, "Return to origin", SETTLE_SEC)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard stop failed: {error._result.result}")

    print("-- Landing")
    await drone.action.land()
    async for state in drone.telemetry.landed_state():
        if state.name == "ON_GROUND":
            print("-- Landed")
            break
    print("-- Done")

    real_baseline = float(np.linalg.norm([
        pos_b.north_m - pos_a.north_m,
        pos_b.east_m - pos_a.east_m,
        pos_b.down_m - pos_a.down_m,
    ]))
    print(f"-- PX4-measured baseline (ground truth): {real_baseline:.3f}m "
          f"(commanded: {BASELINE_NORTH:.3f}m)")

    # Geometric ground truth for frame A, independent of BOTH vision pipelines
    # (ours and aruco_pnp's): the wall's world pose is fixed at (0, 3, 1.5) in
    # kiosk.sdf, and PX4's local NED origin is set at the spawn point (no pose
    # override is passed when px4-rc.gzsim spawns the model), which is world
    # (0, 0, ~). So world_y ~= north_m and the wall-facing distance/lateral
    # follow directly from PX4's own GPS/IMU position estimate -- no camera
    # involved at all, so it stays valid even at ranges where both PnP and our
    # own reconstruction may be degrading.
    geom_forward = 3.0 - pos_a.north_m
    geom_lateral = pos_a.east_m
    print(f"-- GEOMETRIC ground truth (from PX4 position + known wall pose, no vision): "
          f"forward={geom_forward:.3f}m lateral={geom_lateral:.3f}m")

    print("-- Steps 1-4: matching frame A/B")
    _, pts_a, pts_b = detect_and_match(frame_a, frame_b)

    print("-- Steps 5-6: Essential Matrix + triangulation (unit scale)")
    R, t_unit, pts3d_unit = estimate_pose(pts_a, pts_b, K)

    print("-- Step 7: scale correction using PX4 baseline")
    pts3d_m = pts3d_unit * real_baseline

    print("-- Fitting wall plane + camera attitude sanity check")
    normal, centroid, spread = fit_plane(pts3d_m)
    forward = float(-np.dot(normal, centroid))
    lateral, vertical = float(centroid[0]), float(centroid[1])
    yaw_deg = float(np.degrees(np.arctan2(normal[0], -normal[2])))
    cam_rotation_deg = rotation_angle_deg(R)
    confident = spread >= SPREAD_MIN_M

    print(f"-- {len(pts3d_m)} scaled 3D points, spread={spread:.3f}m "
          f"({'CONFIDENT' if confident else f'LOW CONFIDENCE (< {SPREAD_MIN_M}m, likely a single-marker cluster)'})")
    print(f"-- ESTIMATED [Essential] forward={forward:.3f}m lateral={lateral:.3f}m vertical={vertical:.3f}m "
          f"yaw={yaw_deg:+.1f}deg (camera-A frame, plane-fit) CONFIDENT={confident}")
    print(f"-- camera attitude sanity check: rotated {cam_rotation_deg:.1f}deg between A and B "
          f"(commanded yaw=0.0 throughout, so this should be small)")

    print("-- Alternative: Homography-based pose (planar-scene-specific)")
    try:
        R_h, t_h_internal, n_h, inlier_a_h, h_inliers = estimate_pose_homography(pts_a, pts_b, K)
        h_forward, h_lateral, h_vertical, h_yaw_deg, h_spread = homography_to_metric(
            R_h, t_h_internal, n_h, inlier_a_h, K, real_baseline)
        h_cam_rotation_deg = rotation_angle_deg(R_h)
        print(f"-- ESTIMATED [Homography] forward={h_forward:.3f}m lateral={h_lateral:.3f}m "
              f"vertical={h_vertical:.3f}m yaw={h_yaw_deg:+.1f}deg spread={h_spread:.3f}m "
              f"cam_rot={h_cam_rotation_deg:.1f}deg")
    except SystemExit as e:
        print(f"-- Homography pose FAILED: {e}")


def main():
    print("-- Getting camera intrinsics")
    K = capture_camera_info()
    print(f"K=\n{K}")
    asyncio.run(run(K))


if __name__ == "__main__":
    main()
