import time
import math


class Tracker:
    def __init__(self, vehicle, shared,position_estimator):

        self.vehicle = vehicle
        self.shared = shared

        self.camera_image_width = 640
        self.camera_image_height = 480
        self.horizontal_fov = 63

        self.neutral_camera_angle = 60
        self.position_estimator=position_estimator


    def track(self,target):
        print("Tracking started")
        while True:

            
            if target is not None:
                lat,lon= self.position_estimator.get_position(target,track=True)

                print("destlat:", lat)
                print("destlon:", lon)

                self.vehicle.set_roi(lat, lon)

                self.vehicle.move_to(lat, lon)

            if self.shared.mission == "abort":
                self.vehicle.abort_mission()
                break

            time.sleep(.1)
