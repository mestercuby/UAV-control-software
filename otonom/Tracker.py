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

        self.timeout_second = 5

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
        flag = False
        start_timer = time.time()
        while True:
            if not self.shared.track_mission:
                time.sleep(0.1)
                continue
            
            detections, frame, last_update = self.shared.get_detections()
            target_id = self.shared.track_target
            target = None
            if target_id == -1 and len(detections) > 0:
                target = detections[0]
            else:
                for detection in detections:
                    if detection['track_id'] == target_id:
                        print("Tracking:", detection['track_id'])
                        target=detection
                        break
            if target is not None:
                lat, lon = target['position']

                print("destlat:", lat)
                print("destlon:", lon)

                self.vehicle.set_roi(lat, lon)
                self.vehicle.move_to(lat, lon)

                start_timer = time.time()
                flag = True
                
            elif time.time() - start_timer > self.timeout_second and flag:
                self.shared.track_mission = False
                self.vehicle.cancel_roi_mode()
                self.vehicle.set_gimbal_angle(-60,0)
                flag = False
            
            time.sleep(.15)
