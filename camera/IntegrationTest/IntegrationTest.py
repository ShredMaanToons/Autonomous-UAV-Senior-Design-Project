"""
============================
test_flight.py

Mission:
    1. Take off 5 meter above current altitude
    2. Fly back to starting position and land
============================
"""

# ==========================
# IMPORTS
# ==========================
import asyncio
from mavsdk import System

import os                                   # operating system files & folders
import time                                 # time elapsed & pauses program execution
from datetime import datetime               # current date & clock time
from mavsdk.telemetry import LandedState

import cv2 # Import for cv2.undistortPoints for target position
import numpy as np # Import for np.array for target position
import math
from picamera2 import Picamera2

from ultralytics import YOLO

Target_Altitude = 5      # target altitude in meters
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

    # Configure takeoff altitude
    try:
        await action.set_takeoff_altitude(Target_Altitude)
        print(f"Target altitude set: {Target_Altitude:.2f} m")
    except Exception as e:
        print("setting Trarget altitude failed:", type(e).__name__, repr(e))
        return

    # Configure cam
    picam2 = Picamera2()
    cam_config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        controls={
            "FrameRate": 30,
            "ExposureTime": 8000,
            "AnalogueGain": 8.0,
        },
    )
    picam2.configure(cam_config)
    picam2.start()
    await asyncio.sleep(2)
    print(f"Camera Configured")

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

    image_task = asyncio.create_task(image_loop(telemetry, picam2))

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
    image_task.cancel()



# ==========================
# Read Altitude & GPS Position
# ==========================
async def get_attitude(telemetry):
    async for attitude in telemetry.attitude():
        return (attitude.roll, attitude.pitch, attitude.yaw)

# ==========================
# Read Altitude & GPS Position
# ==========================
async def get_position(telemetry):
    async for position in telemetry.position():
        return (position.relative_altitude_m, position.latitude_deg, position.longitude_deg)


async def _target_position(drone_lat,drone_lon, drone_yaw, drone_pitch, drone_roll, altitude, det):
    """Converts the targets pixel offset into an estimated GPS position
    using drone GPS location, pitch, roll, yaw, and altitude"""

    dx,dy = det["error"] # pixels away from center of screen [ direction-> +(> ^) ]

# Improved _target_position
# Intrinsic parameters
    fx = 782.97012129 # Focal length in pixels
    fy = 782.15425781 
    cx = 303.37701841 # Principal point: pixel where the optical axis hits the sensor
    cy = 225.68735933 
# Lens distortion parameters
    k1 = -0.50139567 # Radial distortion: straight lines bowing
    k2 = 0.3469868 # k = 0 for ideal camera
    k3 = -0.27986178
    p1 = 0.00266273 # Tangental distortion: lens not perfectly parallel to sensor
    p2 = 0.00129382 
# Undistorting the position of the detection
    K = np.array([[fx, 0, cx],
                    [0, fy, cy], # Intrinsic matrix, packs the camera calibration data for cv2.undistortPoints
                    [0,  0,  1]], dtype=np.float64)

    dist = np.array([k1, k2, p1, p2, k3], dtype=np.float64) # Packs the distortion coefficients for cv2.undistortPoints

    u, v = dx + cx, -dy + cy # reformat point to originate at top left and have +y be downwards
    pt = np.array([[[u, v]]], dtype=np.float64) # Packs the detection point for cv2.undistortPoints
    xn, yn = cv2.undistortPoints(pt, K, dist)[0, 0] # New points based on ideal pinhole camera, using normalized x,y coordinates

    ray = np.array([-yn, xn, 1.0]) # Creates a ray to use for projecting to ground. -yn so positive is forward


#Acounting for roll, pitch, and yaw
# pitch, roll, and yaw degrees to radians
    PITCH_RAD = math.radians(drone_pitch)
    ROLL_RAD = math.radians(drone_roll)
    YAW_RAD = math.radians(drone_yaw) # Positive is clockwise as seen from above, the image the camera sees spins opposite
# variables to represent sin and cos of each rotation
    cos_y, sin_y = math.cos(YAW_RAD), math.sin(YAW_RAD)
    cos_p, sin_p = math.cos(PITCH_RAD), math.sin(PITCH_RAD)
    cos_r, sin_r = math.cos(ROLL_RAD), math.sin(ROLL_RAD)

    Rx = [[1,   0,    0 ],    # roll: rotate about forward axis
        [0,  cos_r,  -sin_r ],
        [0,  sin_r,   cos_r ]]

    Ry = [[ cos_p, 0,  sin_p ],    # pitch: rotate about right axis
        [  0, 1,   0 ],
        [-sin_p, 0,  cos_p ]]

    Rz = [[cos_y, -sin_y, 0 ],      # yaw: rotate about down axis
        [sin_y,  cos_y, 0 ],
        [ 0,   0, 1 ]]

    ray_ned = Rz @ Ry @ Rx @ ray # Matrix multiplication combine the rotations with the ray

    if ray_ned[2] <= 0.05:  # Ray is pointing too close to or above the horizon
        return None
    scale = altitude / ray_ned[2] # Find the relationship between the altitude and the vertical ray component.
    meter_north = ray_ned[0] * scale # Scale the north/south ray component to the meter amount
    meter_east  = ray_ned[1] * scale # Scale the east/west ray component to the meter amount

# Add latitude and longitude to drone latitude and longitude
    obj_lat = drone_lat + (meter_north / 111320)
    obj_lon = drone_lon + (meter_east / (111320 * math.cos(math.radians(drone_lat))))

    return obj_lat, obj_lon


async def mult_detect_targets(frame, model, classes, conf):
    # Detects the target then gives the bounding box
    # Returns the box offset from the center of frame

    if frame is None or frame.size == 0:
        return None, frame # Rejects invalid images.

    # model is a YOLO object, (Frame is the input image, verbose=False supresses console output, conf is confidence, classes is the list of objects we are identifying??)
    # [0] at the end is used to change the default array output of the model into the first scalar item from the array which is put into "res".
    # res is "result", taking the result of the analized frame
    # res.boxes returns the box of the image
    # res.names[id] returns the name of the boxed item associated with the class id parameter
    res = model(frame, verbose=False, conf=conf, classes=classes)[0]
    # Try if lagging res = await asyncio.to_thread(model, frame)

    # Finding center of the input image
    H, W = frame.shape[:2]
    frame_cx = W / 2.0
    frame_cy = H / 2.0

    targets = []

    # Loop through every box in the image, noting the 1) confidence, 2) location in frame, 3) width and hight, 4) Center, class info, and stores the info in the targets array that will be returned
    for b in res.boxes:
        # 1)
        det_conf = float(b.conf.item())
        # 2)
        x1, y1, x2, y2 = b.xyxy[0].cpu().numpy()
        # 3)
        x = int(x1)
        y = int(y1)
        w = int(x2 - x1)
        h = int(y2 - y1)
        
        if w <= 0 or h <= 0:
            continue # Skip broken boxes
        # 4)
        cx = x + w / 2.0
        cy = y + h / 2.0
        err_px = (cx - frame_cx, cy - frame_cy)

        # Get class info
        class_id = int(b.cls[0])
        class_name = res.names[class_id]

        targets.append({
            "bbox": (x,y,w,h),
            "class": class_name,
            "error": err_px,
            "conf": det_conf
        })

    return targets, frame


async def imageProcess(telemetry, picam2):
    [curAltitude, curLatitude, curLongitude] = await get_position(telemetry)
    [curRoll, curPitch, curYaw] = await get_attitude(telemetry)
    frame = await asyncio.to_thread(picam2.capture_array)
    confidence = 0.5
    classes = None
    model = YOLO() # Insert model path
    detections, _ = mult_detect_targets(frame, model, classes, conf=confidence)
    if detections:
         for det in detections:
            [obj_lat, obj_lon] = _target_position(curLatitude, curLongitude, curYaw, curPitch, curRoll, curAltitude, det)
            print(f"object latitude: {obj_lat:.2f} ")
            print(f"object longitude: {obj_lon:.2f} ")

async def image_loop(telemetry, picam2):
    while True:
        await imageProcess(telemetry, picam2)
        await asyncio.sleep(0.75)

if __name__ == "__main__":
    asyncio.run(main())