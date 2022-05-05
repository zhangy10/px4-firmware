#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.offboard import (PositionNedYaw, VelocityNedYaw, OffboardError)
import os

async def run():

    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_velocity_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0),VelocityNedYaw(0.0,0.0,0.0,0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
   
    print("-- Go 5m UP within local coordinate system")
    await drone.offboard.set_position_velocity_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0),VelocityNedYaw(0.0,0.0,1.0,0.0))
    await asyncio.sleep(10)

    print("-- Go 5m North within local coordinate system")
    await drone.offboard.set_position_velocity_ned(PositionNedYaw(5.0, 0.0, 0.0, 0.0),VelocityNedYaw(1.0,0.0,0.0,0.0))
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    await drone.action.land()
    print("-- Landing")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
