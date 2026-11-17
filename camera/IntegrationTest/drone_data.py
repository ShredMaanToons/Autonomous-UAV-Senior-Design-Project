
"""
==========================
drone_data.py
==========================
This file provides the basic functions needed by mission files.

Handles:
    - Battery Percentage
    - simple flight logging
==========================
"""

# ==========================
# Imports
# ==========================
import os                                   # operating system files & folders
import time                                 # time elapsed & pauses program execution
import asyncio                              # 
from datetime import datetime               # current date & clock time
from mavsdk import System                   #
from mavsdk.telemetry import LandedState

LOG_DIR = "mission/flight_logs"             # flight_logs folder path 
LOG_INTERVAL = 30                           # how often it logs in seconds

# ==========================
# Connect to Pixhawk
# ==========================
async def connect_drone():
    drone = System()
    print("Connecting to Pixhawk")
    await drone.connect(system_address="serial:///dev/ttyAMA0:921600")
    print("Waiting for drone connection")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break
    return drone

# ==========================
# Arm Drone
# ==========================
async def arm_drone(drone):
    try:
        await drone.action.arm()
        print("Drone Armed")
        return True
    except Exception as e:
        print("Arming failed:", type(e).__name__, repr(e))
        return False

# ==========================
# Disarm Drone
# ==========================
async def disarm_drone(drone):
    try:
        async for landed_state in drone.telemetry.landed_state():
            if landed_state != LandedState.ON_GROUND:
                print("Land Drone to Disarm")
                return False
            break
        await drone.action.disarm()
        print("Drone Disarmed")
        return True
    except Exception as e:
        print("Disarming failed:", type(e).__name__, repr(e))
        return False

# ==========================
# Read battery percentage
# ==========================
async def get_battery(telemetry):
    battery = await telemetry.battery()
    return battery.remaining_percent * 100

# ==========================
# Read Altitude & GPS Position
# ==========================
async def get_position(telemetry):
    async for position in telemetry.position():
        return (position.relative_altitude_m,position.latitude_deg,position.longitude_deg)

# ==========================
# Read Roll Pitch Yaw
# ==========================
async def get_attitude(telemetry):
    async for attitude in telemetry.attitude():
        return (attitude.relative_altitude_m,position.latitude_deg,position.longitude_deg)

# ==========================
# Flight Log
# ==========================
async def start_flight_log(telemetry):
    os.makedirs(LOG_DIR, exist_ok=True)

    # Creates name for current flight
    file_time = datetime.now()
    file_name = file_time.strftime("flight_log_%m-%d-%Y_%H:%M.txt")
    file_log = os.path.join(LOG_DIR, file_name)

    flight_timer = time.monotonic()
    print("Flight Log started.")

    # Creates new file for flight
    with open(file_log, "x", encoding="utf-8") as file:
        file.write(f"Flight Started: {datetime.now()}\n")
        file.write("flight_time,battery_percentage,altitude_m,latitude,longitude\n")

    try:
        while True:

            # Functions Displayed in Flight Log
            battery = await get_battery(telemetry)
            altitude, latitude, longitude = await get_position(telemetry)

            # Keeps track of time passed
            total_sec = int(time.monotonic() - flight_timer)
            hr = total_sec // 3600
            min = (total_sec % 3600) // 60
            sec = total_sec % 60
            timestamp = f"{hr:02}:{min:02}:{sec:02}"

            # Writes information in file per set interval
            with open(file_log, "a", encoding="utf-8") as file:
                file.write(f"{timestamp},{battery:.1f}%,{altitude:.2f},{latitude:.7f},{longitude:.7f}\n")
            await asyncio.sleep(LOG_INTERVAL)

    except asyncio.CancelledError:
        print("Flight logger stopped.")
        raise
