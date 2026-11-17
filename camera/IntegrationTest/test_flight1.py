"""
============================
test_flight.py

Mission:
    1. Take off 1.5 meter above current altitude
    2. Fly back to starting position and land
============================
"""

# ==========================
# IMPORTS
# ==========================
import asyncio
from mavsdk import System

Target_Altitude = 1.5      # target altitude in meters
Hover_Time = 10             # Hover time in seconds

async def main():
    drone = System()
    print("Connecting to Pixhawk")

    # await drone.connect(system_address="udpin://0.0.0.0:14540")
    await drone.connect(system_address="serial:///dev/ttyAMA0:921600")
    print("Waiting for drone connection")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break

    telemetry = drone.telemetry
    action = drone.action

    # Not needed but observing altitude data
    # Wait for a altitude data
    async for position in telemetry.position():
        current_altitude = position.relative_altitude_m
        print(f"Current altitude: {current_altitude:.2f} m")
        break
    # Target altitude
    new_altitude = current_altitude + 1.5
    print(f"Target altitude: {new_altitude:.2f} m")
    #

    # Configure takeoff altitude
    try:
        await action.set_takeoff_altitude(Target_Altitude)
        print(f"Target altitude set: {Target_Altitude:.2f} m")
    except Exception as e:
        print("setting Trarget altitude failed:", type(e).__name__, repr(e))
        return

    # Arm drone
    try:
        await action.arm()
        print(f"Drone armed")
    except Exception as e:
        print("Arming failed:", type(e).__name__, repr(e))
        return

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

    # Returns to home position and land
    await action.return_to_launch()

if __name__ == "__main__":
    asyncio.run(main())