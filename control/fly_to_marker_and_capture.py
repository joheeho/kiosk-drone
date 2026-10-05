#!/usr/bin/env python3
"""
Fly x500_depth to a hover position ~0.6m in front of the kiosk_wall ArUco marker
(wall at world (0,3,1.5)), hold, run capture_and_detect.py during the hold, then land.

NED offsets are world-frame (north = world +Y, east = world +X), independent of heading.
PositionNedYaw's yaw is an absolute NED bearing, not a rotation relative to spawn heading.
Spawn yaw is irrelevant; NED yaw=0 = north = facing the wall directly. The vehicle holds
yaw=0 throughout, then moves north toward the marker while facing it.
"""

import asyncio
from pathlib import Path

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

CAPTURE_SCRIPT = Path(__file__).resolve().parent.parent / "vision" / "capture_and_detect.py"

ALT = -1.5      # NED z for 1.5m altitude (matches wall center height)
NORTH_TARGET = 1.2  # 0.6m stand-off from wall at y=3
YAW_TO_MARKER = 0.0  # degrees; spawn yaw is irrelevant, NED yaw 0 = north = wall direction
HOLD_SEC = 6
SETTLE_SEC = 4


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
    await goto(drone, 0.0, 0.0, YAW_TO_MARKER, f"Rotate to yaw={YAW_TO_MARKER} (facing wall)", SETTLE_SEC)
    await goto(drone, NORTH_TARGET, 0.0, YAW_TO_MARKER, f"North {NORTH_TARGET}m (~0.6m from wall), facing marker", SETTLE_SEC)

    print(f"-- Holding for {HOLD_SEC}s, running capture_and_detect.py")
    capture_task = asyncio.create_task(
        asyncio.create_subprocess_exec(
            "/usr/bin/python3", str(CAPTURE_SCRIPT),
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
        )
    )
    await asyncio.sleep(HOLD_SEC)
    proc = await capture_task
    out, _ = await proc.communicate()
    print(out.decode())

    print("-- Returning to origin")
    await goto(drone, 0.0, 0.0, YAW_TO_MARKER, "Return to origin", SETTLE_SEC)

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


if __name__ == "__main__":
    asyncio.run(run())
