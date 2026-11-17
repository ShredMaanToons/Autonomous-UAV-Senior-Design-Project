"""
============================
test_flight2.py

Mission:
    1. Take off 1.5 meter above current altitude
    2. Fly back to starting position and land

Controls:
    Press 1 + Enter = Start/Stop flight logger
    Press 2 + Enter = Arm/Disarm
    Press 3 + Enter = Start Mission
============================
"""

# ==========================
# Imports
# ==========================
import asyncio
from aioconsole import ainput
from mavsdk.telemetry import LandedState
from preflight import preflight_check
from drone_data import (connect_drone,start_flight_log,arm_drone,disarm_drone)

Target_Altitude = 1.5      # target altitude in meters
Hover_Time = 10             # Hover time in seconds

# ==========================
# Keyboard Controls
# ==========================
async def key_control(drone, mission_start_event):
    telemetry = drone.telemetry
    logger_task = None
    print("1: Start/Stop flight log\n")
    print("2: Arm/Disarm\n")
    print("3: Start Mission")

    try:
        while True:
            key = await ainput()

            # Key 1: Flight log
            if key.strip() == "1":

                # Start logger
                if logger_task is None:
                    logger_task = asyncio.create_task(start_flight_log(telemetry))
                # Stop logger
                else: 
                    logger_task.cancel()
                    try:
                        await logger_task
                    except asyncio.CancelledError:
                        pass
                    logger_task = None

            # Key 2: Arm/Disarm
            elif key.strip() == "2":
                async for is_armed in telemetry.armed():
                    break
                if not is_armed:
                    await arm_drone(drone)
                else:
                    await disarm_drone(drone)

            # Key 3: Start Mission
            elif key.strip() == "3":
                if mission_start_event.is_set():
                    print("Mission already started")
                    continue
                async for is_armed in telemetry.armed():
                    break
                if not is_armed:
                    print("Drone Not Armed, Start Mission Failed")
                    continue
                print("Mission Started")
                mission_start_event.set()

    finally:
        if logger_task is not None:
            logger_task.cancel()
            try:
                await logger_task
            except asyncio.CancelledError:
                pass


# ==========================
# Mission
# ==========================
async def main():
    # Connects to Pixhawk
    drone = await connect_drone()

    telemetry = drone.telemetry
    action = drone.action

    # Preflight data
    await preflight_check(drone)

    mission_start_event = asyncio.Event()

    # Monitoring Keyboard Inputs
    key_control_task = asyncio.create_task(key_control(drone))

    try:

        # Configure takeoff altitude
        try:
            await action.set_takeoff_altitude(Target_Altitude)
            print(f"Target altitude set: {Target_Altitude:.2f} m")
        except Exception as e:
            print("Setting Trarget altitude failed:", type(e).__name__, repr(e))
            return

        # Starts Mission if Drone is Armed
        await mission_start_event.wait()

        # Take off
        try:
            await action.takeoff()
            print(f"Takeoff command accepted")
        except Exception as e:
            print("Takeoff failed:", type(e).__name__, repr(e))
            return

        # Monitor altitude until target is reached
        async for position in telemetry.position():

            altitude = position.relative_altitude_m
            print(f"Current altitude: {altitude:.2f} m")

            if altitude >= Target_Altitude - 0.3:
                print("Target altitude reached")
                break

        # Hover at target altitude
        await asyncio.sleep(Hover_Time)

        print("Returning to Launch Location")
        await action.return_to_launch()

    finally:
        # Stop Keyboard Inputs
        key_control_task.cancel()

        try:
            await key_control_task
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    asyncio.run(main())