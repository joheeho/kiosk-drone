#!/usr/bin/env python3
"""Position sweep for the mono-multiview baseline test.

Flies a single approach pass north, capturing a frame + ORB keypoint count at
each of SWEEP_NORTH so we can pick two positions for
mono_multiview_capture_and_match.py's APPROACH_NORTH/BASELINE_NORTH that
actually keep the kiosk wall fully framed (same validate-by-measurement
approach as the aruco-pnp branch's distance sweep, rather than guessing).

TEMPORARY experiment script, not production.
"""

import asyncio
from pathlib import Path

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
import gz.transport13 as gz_transport
import threading
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

RGB_TOPIC = "/world/kiosk/model/x500_depth_0/link/camera_link/sensor/IMX214/image"
OUT_DIR = Path(__file__).resolve().parent.parent / "outputs" / "sweep"
OUT_DIR.mkdir(parents=True, exist_ok=True)

ALT = -1.5
SWEEP_NORTH = [0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4]
SETTLE_SEC = 3


def capture_frame(tag):
    frame = {}
    got = threading.Event()

    def cb(msg: Image):
        if got.is_set():
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = arr.reshape((msg.height, msg.width, 3))
        frame["img"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        got.set()

    node = gz_transport.Node()
    node.subscribe(Image, RGB_TOPIC, cb)
    if not got.wait(timeout=10):
        print(f"  !! capture failed at {tag}")
        return None
    img = frame["img"]
    cv2.imwrite(str(OUT_DIR / f"sweep_{tag}.png"), img)
    return img


def keypoint_count(img):
    orb = cv2.ORB_create(nfeatures=2000)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kp = orb.detect(gray, None)
    return len(kp)


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

    await goto(drone, 0.0, 0.0, 0.0, "Takeoff to 1.5m", 4)

    loop = asyncio.get_running_loop()
    results = []
    for north in SWEEP_NORTH:
        await goto(drone, north, 0.0, 0.0, f"North {north}m", SETTLE_SEC)
        img = await loop.run_in_executor(None, capture_frame, f"{north:.1f}")
        n_kp = keypoint_count(img) if img is not None else -1
        print(f"  north={north:.1f}m  keypoints={n_kp}")
        results.append((north, n_kp))

    print("-- Returning to origin")
    await goto(drone, 0.0, 0.0, 0.0, "Return to origin", 4)

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

    print("\n=== SWEEP SUMMARY ===")
    for north, n_kp in results:
        print(f"  north={north:.1f}m  keypoints={n_kp}")


if __name__ == "__main__":
    asyncio.run(run())
