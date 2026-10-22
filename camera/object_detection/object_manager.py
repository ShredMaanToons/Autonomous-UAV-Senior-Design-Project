import math
import json # For saving the data to a file

class ObjectManager:
    # When creating on object from this class automatically automatically creates an empty array to store the objects and sets it distance and detection threshold
    def __init__(self, distance_thresh_m=2.5, min_detections=2):
        self.objects = []
        self.distance_thresh = distance_thresh_m
        self.min_detections = min_detections

    # Method to feed the drone position and object information into the other methods
    def update(self, detections, drone_gps, altitude):
        """Feeds detected objects and the current drone position into methods for finding the object position, matching it to a previous detection, or creating a new detection"""
        lat, lon = drone_gps

        for det in detections:
            obj_lat, obj_lon = self._target_position(lat, lon, altitude, det)   # run method to determine object position

            match = self._find_match(det["class"], obj_lat, obj_lon)   # run method to determine the index of the first object of the same class that is within the distance threshold

            if match is not None:
                self._update_object(match, obj_lat, obj_lon, det["conf"])   # if there was a match we run the method to update the matched object's position and confidence data
            else:
                self._create_object(det["class"], obj_lat, obj_lon, det["conf"])   # if there was no match we run the method to create a new object.

        self.save_objects()   # Write the list of objects to objects.txt


    # Write the list of objects to objects.txt
    def save_objects(self, file="objects.txt"):
        with open(file, "w") as f:
            json.dump(self.objects, f, indent=4)

# Rewrite of _target_position
    def _target_position(self, drone_lat,drone_lon, drone_yaw, drone_pitch, drone_roll, altitude, det):
        """Converts the targets pixel offset into an estimated GPS position
        using drone GPS location, pitch, roll, yaw, and altitude"""

        dx,dy = det["error"] # pixels away from center of screen [ direction-> +(> ^) ]

        # Find total area camera can see
        FOVH = math.radians(50) # Radians horizontal
        FOVV = math.radians(38) # Radians vertical
        # Find radian based angle of ray to object: -----
        ANGH = (dx/640) * (FOVH) # Angle horizontal = ratio of pixels from center to all pixels (left -0.5 to 0.5 right) multiplied by total radians
        ANGV = (dy/480) * (FOVV) # Angle vertical = ratio of pixels from center to all pixels (bottom -0.5 to 0.5 top) multiplied by total radians
        
        # adjust angle based on pitch and roll: -----
        # pitch and roll degrees to radians
        PITCH_RAD = math.radians(drone_pitch)
        ROLL_RAD = math.radians(drone_roll)
        # ADD pitch and roll radians to final radian angle of ray to object
        ANGV += PITCH_RAD # Positive pitch is nose up (so add)
        ANGH -= ROLL_RAD # Positive roll is banking to the right (so subtract)

        # Change angle into distance on ground: -----
        METER_V = altitude * math.tan(ANGV) #(+ ^)
        METER_H = altitude * math.tan(ANGH) #(+ >)

        # Rotate for yaw: -----
        # change yaw to radians
        YAW_RAD = math.radians(drone_yaw) # Positive is clockwise as seen from above, the image the camera sees spins opposite
        # rotate
        METER_EAST = (METER_H * math.cos(YAW_RAD)) + (METER_V * math.sin(YAW_RAD)) # Left right
        METER_NORTH = -(METER_H * math.sin(YAW_RAD)) + (METER_V * math.cos(YAW_RAD)) # Up down

        # Add latitude and longitude to drone latitude and longitude
        obj_lat = drone_lat + (METER_NORTH / 111320)
        obj_lon = drone_lon + (METER_EAST / (111320 * math.cos(math.radians(drone_lat))))

        return obj_lat, obj_lon
        # Double check that the inputs are in degrees
        # IMPROVEMENTS -------------------------------------
        # * Non-linear camera
        # * Possibly need to couple pitch and roll (Cannot be calculated seperately?)
        # * Edge case exceptions for tan
        # * Impliment timestamp
        
    def _find_match(self, target_class, lat, lon):
        """Searches stored objects for the same class within the distance threshold"""

        for i, obj in enumerate(self.objects): # Skips all objects that are a different class
            if obj["class"] != target_class:
                continue
            dist = self._gps_distance(lat,lon,obj["lat"], obj["lon"]) # run method to determine distance between stored object and detected object

            if dist < self.distance_thresh: # indicate if it is within the minimum distance of a previously detected object of the same class
                return i
        return None
    
    def _update_object(self, idx, lat, lon, conf):
        # Averages a repeated detection's GPS location and update confidence and count
        obj = self.objects[idx]

        count = obj["count"] # read the amound of times this object has been seen into a variable

        obj["lat"] = (obj["lat"] * count + lat) / (count + 1)
        obj["lon"] = (obj["lon"] * count + lon) / (count + 1)
        obj["conf"] = max(obj["conf"], conf) # set the confidence to the highest confidence from all detections
        obj["count"] += 1 # Count the new object


    def _create_object(self, target_class, lat, lon, conf):
        """Adds a newly detected target to the object list"""
        self.objects.append({
            "class": target_class,
            "lat": lat,
            "lon": lon,
            "conf": conf,
            "count": 1
        })


    def _gps_distance(self,lat1,lon1,lat2,lon2):
        """Converts two GPS coordinates into ground distance in meters for duplicate target matching"""
        dx = (lon2 - lon1) * 111111 * math.cos(math.radians(lat1))
        dy = (lat2 - lat1) * 111111
        return math.sqrt(dx**2 + dy**2)
        
    def get_confirmed_objects(self):
        """Returns only targets detected a minimum number of times to reduce false positives"""
        confirmed = []

        for obj in self.objects:
            if obj["count"] >= self.min_detections:
                confirmed.append(obj)

        return confirmed
