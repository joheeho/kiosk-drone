#!/usr/bin/env python3
"""Two-frame monocular baseline test for the 이동 구간 (movement zone) 방향 B.

Captures frame A, moves the drone forward a fixed step, captures frame B,
then runs ORB detection + BFMatcher + RANSAC (Fundamental matrix) to check
whether the matches are clean enough to build Essential-matrix pose recovery
and triangulation on top of (steps 5-8 of the mono-multiview plan).

TEMPORARY validation script, not the production node — see fly_to_marker_and_capture.py
for the pattern this will move to once matching is proven reliable: a persistent
ROS2 node subscribing to the camera continuously, like aruco_pnp_node.py, rather
than a one-shot script. BASELINE_NORTH is hardcoded for now; production will read
the real frame-to-frame displacement from PX4 vehicle_local_position for scale
correction (step 7) instead.
"""

import asyncio
import threading
from pathlib import Path

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
import gz.transport13 as gz_transport
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

RGB_TOPIC = "/world/kiosk/model/x500_depth_0/link/camera_link/sensor/IMX214/image"
OUT_DIR = Path(__file__).resolve().parent.parent / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

ALT = -1.5           # NED z for 1.5m altitude
# Chosen from mono_multiview_position_sweep.py's ORB keypoint-count-vs-distance
# measurement (north<=0.8m: wall out of frame, 0 keypoints; peak ~609kp at 1.6m):
# 1.4m and 2.0m both land solidly in the well-framed range (450kp / 335kp).
APPROACH_NORTH = 1.4
BASELINE_NORTH = 0.6  # frame A -> frame B forward step [m], hardcoded for this test
SETTLE_SEC = 4


def capture_frame(tag):
    """Blocking single-frame grab via gz-transport (same pattern as capture_and_detect.py)."""
    frame = {}
    got = threading.Event()

    def cb(msg: Image):
        if got.is_set():
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = arr.reshape((msg.height, msg.width, 3))  # RGB_INT8
        frame["img"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        got.set()

    node = gz_transport.Node()
    node.subscribe(Image, RGB_TOPIC, cb)
    if not got.wait(timeout=10):
        raise SystemExit(f"failed to capture frame {tag}")

    img = frame["img"]
    cv2.imwrite(str(OUT_DIR / f"mono_frame_{tag}.png"), img)
    print(f"saved {OUT_DIR / f'mono_frame_{tag}.png'}  shape={img.shape}")
    return img


def detect_and_match(img_a, img_b):
    orb = cv2.ORB_create(nfeatures=2000)
    gray_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
    gray_b = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY)
    kp_a, des_a = orb.detectAndCompute(gray_a, None)
    kp_b, des_b = orb.detectAndCompute(gray_b, None)
    print(f"keypoints: A={len(kp_a)} B={len(kp_b)}")

    if des_a is None or des_b is None or len(kp_a) < 8 or len(kp_b) < 8:
        raise SystemExit("too few keypoints to match")

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = sorted(bf.match(des_a, des_b), key=lambda m: m.distance)
    print(f"raw matches: {len(matches)}")

    if len(matches) < 8:
        raise SystemExit("too few matches for RANSAC (need >= 8)")

    pts_a = np.float32([kp_a[m.queryIdx].pt for m in matches])
    pts_b = np.float32([kp_b[m.trainIdx].pt for m in matches])

    _, mask = cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_RANSAC, 1.0, 0.99)
    inlier_count = int(mask.sum()) if mask is not None else 0
    print(f"RANSAC inliers: {inlier_count}/{len(matches)} "
          f"({100 * inlier_count / len(matches):.1f}%)")

    inlier_matches = [m for m, keep in zip(matches, mask.ravel()) if keep]
    vis = cv2.drawMatches(img_a, kp_a, img_b, kp_b, inlier_matches[:60], None,
                           flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(OUT_DIR / "mono_matches.png"), vis)
    print(f"saved {OUT_DIR / 'mono_matches.png'}")

    print(f"MATCH_SUCCESS={inlier_count >= 20}")
    return inlier_matches


async def goto(drone, north, east, yaw, label, hold):
    print(f"-- {label}")
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, ALT, yaw))
    await asyncio.sleep(hold)


async def run():
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
    await goto(drone, APPROACH_NORTH, 0.0, 0.0, f"North {APPROACH_NORTH}m (approach, wall in frame)", SETTLE_SEC)

    print("-- Capturing frame A")
    loop = asyncio.get_running_loop()
    frame_a = await loop.run_in_executor(None, capture_frame, "a")

    baseline_target = APPROACH_NORTH + BASELINE_NORTH
    await goto(drone, baseline_target, 0.0, 0.0, f"North {baseline_target}m (baseline step)", SETTLE_SEC)

    print("-- Capturing frame B")
    frame_b = await loop.run_in_executor(None, capture_frame, "b")

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

    print("-- Matching frame A/B")
    detect_and_match(frame_a, frame_b)


if __name__ == "__main__":
    asyncio.run(run())
