#!/usr/bin/env python3
"""Distance sweep for the full mono-multiview pipeline (steps 1-7 + plane fit
angle). Visits a series of north positions in one flight, capturing a frame +
position at each, then runs the complete match -> Essential Matrix -> triangulate
-> scale -> plane-fit pipeline on every CONSECUTIVE pair to map out where
results start becoming unreliable -- rather than guessing a single "good"
distance, follow the same measure-first approach used for the earlier position
sweep and the aruco-pnp branch's own validation.

Each pair's estimated forward distance is checked against the wall's known
world pose (kiosk.sdf: (0, 3, 1.5)) combined with PX4's own position estimate
-- a ground truth independent of both our vision pipeline and aruco_pnp's, so
it stays valid even at ranges where the vision-based methods may be degrading.

TEMPORARY experiment script, not production.
"""

import asyncio

import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

from mono_multiview_capture_and_match import (
    ALT, SETTLE_SEC, capture_frame, detect_and_match, goto,
)
from mono_multiview_pose_estimate import (
    SPREAD_MIN_M, bbox_center_lateral_vertical, capture_camera_info, estimate_pose,
    fit_plane, rotation_angle_deg,
)

WALL_WORLD_Y = 3.0  # kiosk.sdf: <pose>0 3 1.5 0 0 0</pose>
SWEEP_NORTH = [0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.2]


async def get_position_ned(drone):
    async for pv in drone.telemetry.position_velocity_ned():
        return pv.position


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

    loop = asyncio.get_running_loop()
    frames, positions = [], []
    for north in SWEEP_NORTH:
        await goto(drone, north, 0.0, 0.0, f"North {north}m", SETTLE_SEC)
        img = await loop.run_in_executor(None, capture_frame, f"{north:.1f}")
        pos = await get_position_ned(drone)
        frames.append(img)
        positions.append(pos)
        print(f"  captured target={north:.2f}m actual_north={pos.north_m:.3f}m")

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

    print("\n=== PAIRWISE PIPELINE RESULTS ===")
    for i in range(len(frames) - 1):
        img_a, img_b = frames[i], frames[i + 1]
        pos_a, pos_b = positions[i], positions[i + 1]
        tag = f"[{SWEEP_NORTH[i]:.1f}->{SWEEP_NORTH[i+1]:.1f}]"
        geom_forward = WALL_WORLD_Y - pos_a.north_m
        real_baseline = float(np.linalg.norm([
            pos_b.north_m - pos_a.north_m,
            pos_b.east_m - pos_a.east_m,
            pos_b.down_m - pos_a.down_m,
        ]))
        try:
            _, pts_a, pts_b = detect_and_match(img_a, img_b)
            R, t_unit, pts3d_unit, inlier_2d_a = estimate_pose(pts_a, pts_b, K)
            pts3d_m = pts3d_unit * real_baseline
            normal, centroid, spread = fit_plane(pts3d_m)
            forward = float(-np.dot(normal, centroid))
            lateral, vertical = bbox_center_lateral_vertical(inlier_2d_a, K, normal, forward)
            yaw_deg = float(np.degrees(np.arctan2(normal[0], -normal[2])))
            cam_rot = rotation_angle_deg(R)
            confident = spread >= SPREAD_MIN_M
            err_pct = abs(forward - geom_forward) / geom_forward * 100
            print(f"{tag} geom_forward={geom_forward:.2f}m est_forward={forward:.2f}m (err {err_pct:.0f}%) "
                  f"lateral={lateral:+.2f}m yaw={yaw_deg:+.1f}deg spread={spread:.2f}m "
                  f"CONFIDENT={confident} cam_rot={cam_rot:.1f}deg baseline={real_baseline:.2f}m")
        except SystemExit as e:
            print(f"{tag} FAILED: {e}")


def main():
    print("-- Getting camera intrinsics")
    K = capture_camera_info()
    asyncio.run(run(K))


if __name__ == "__main__":
    main()
