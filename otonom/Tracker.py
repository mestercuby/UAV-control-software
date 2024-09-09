import time
import math


class Tracker:
    def __init__(self, vehicle, shared, position_estimator):

        self.vehicle = vehicle
        self.shared = shared

        self.camera_image_width = 640
        self.camera_image_height = 480
        self.horizontal_fov = 63

        self.neutral_camera_angle = 60
        self.position_estimator = position_estimator

    def get_distance_from_lat_lon_in_km(point1, point2):
        R = 6371  # Radius of the earth in km
        dLat = math.radians(point2[0] - point1[0])
        dLon = math.radians(point2[1] - point1[1])
        a = (
                math.sin(dLat / 2) * math.sin(dLat / 2) +
                math.cos(math.radians(point1[0])) * math.cos(math.radians(point2[0])) *
                math.sin(dLon / 2) * math.sin(dLon / 2)
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = R * c  # Distance in km
        return d

    def track(self):
        while True:
            if not self.shared.track_mission:
                time.sleep(0.1)
                continue
            
            detections = self.shared.get_detections()
            target_id = self.shared.target
            target = None
            for detection in detections:
                if detection['id'] == target_id:
                    print("Tracking:", detection['id'])
                    target=detection
                    break
            if target is not None:
                lat, lon = target['position']

                print("destlat:", lat)
                print("destlon:", lon)

                self.vehicle.set_roi(lat, lon)
                self.vehicle.move_to(lat, lon)
            else:
                print("Target lost")
            
            time.sleep(.15)
