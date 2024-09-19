import time
import math
from collections import deque
import numpy as np

class Tracker:
    def __init__(self, vehicle, shared, position_estimator, enablemove=False, enableroi=False):

        self.vehicle = vehicle
        self.shared = shared
        self.enablemove=enablemove
        self.enableroi=enableroi

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

    def filter_outliers_rolling_iqr(self,stream, window_size=5, iqr_factor=5):
        rolling_window = deque(maxlen=window_size)
        filtered_stream = []
        print("Stream:", stream)
        for value in stream:
            if len(rolling_window) < window_size:
                rolling_window.append(value)
                filtered_stream.append(value)
            else:
                # Calculate rolling median and IQR
                window_median = np.median(rolling_window)
                q1 = np.percentile(rolling_window, 25)
                q3 = np.percentile(rolling_window, 75)
                iqr = q3 - q1

                # Define dynamic threshold
                lower_bound = window_median - iqr_factor * iqr
                upper_bound = window_median + iqr_factor * iqr

                if lower_bound <= value <= upper_bound:
                    filtered_stream.append(value)
                    rolling_window.append(value)
                else:
                    filtered_stream.append(None)  # Mark as outlier
        
        return filtered_stream[-1]
    
    def track(self):
        flag = False
        start_timer = time.time()
        olddet = None
        olddists = []
        while True:
            if not self.shared.track_mission:
                time.sleep(0.1)
                continue
            
            detections, frame, last_update = self.shared.get_detections()
            target_id = self.shared.track_target
            target = None
            if target_id == -1 and len(detections) > 0 and olddet ==None:
                target = detections[0]
                olddet = target
            elif target_id == -1 and len(detections) > 0:
                for detection in detections:
                    if detection['track_id'] == olddet['track_id']:
                        target=detection
                        break
                if target is None:
                    target = detections[0]
                    olddet = target
            else:
                for detection in detections:
                    if detection['track_id'] == target_id:
                        target=detection
                        break

            if target is not None:
                print("Tracking:", target['track_id'])
                lat, lon = target['position']
                distance = target['distance']
                
                olddists.append(distance)

                if self.filter_outliers_rolling_iqr(olddists) == None:
                    print("distance outlier : ", distance)
                    olddists.pop(-1)
                    continue
                    
                if self.enableroi:
                    self.vehicle.set_roi(lat, lon)
                if self.enablemove: 
                    self.vehicle.move_to(lat, lon)
                start_timer = time.time()
                flag = True
                time.sleep(0.7)
            elif time.time() - start_timer > self.timeout_second and flag:
                self.shared.track_mission = False
                self.vehicle.cancel_roi_mode()
                self.vehicle.set_gimbal_angle(-45,0)
                flag = False
            
            time.sleep(.15)
