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

    # Will be rewritten to account for pitch and yaw
    def _target_position(self, drone_lat,drone_lon,altitude, det):
        """Converts the targets pixel offset into an estimated GPS position
        using drone GPS location and altitude"""

        dx,dy = det["error"]

        # Find total area camera can see
        FOVW = math.radians(50)
        FOVL = math.radians(38)
        W = 2*altitude * math.tan(FOVW/2)
        L = 2*altitude * math.tan(FOVL/2)

        # Compute meters per pixel in X and Y directions
        H_scale = W/640
        V_scale = L/480

        # Convert pixel offsets to real-world GPS offsets (meters → degrees)
        dlat = dy*V_scale
        dlon = dx*H_scale
        obj_lat = drone_lat + (dlat / 111320)
        obj_lon = drone_lon + (dlon / (111320 * math.cos(math.radians(drone_lat))))

        return obj_lat, obj_lon
        
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
