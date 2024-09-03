import time
import math


class Tracker:
    def __init__(self, vehicle, shared):

        self.vehicle = vehicle
        self.shared = shared

    def hesap(self, height, pos, horizontal_fov, camera_angle):
        pixelwidth = 640
        pixelheight = 480
        
        aspect_ratio = float(pixelwidth) / pixelheight
        horizontal_fov_rad = math.radians(horizontal_fov)
        camera_angle_rad = math.radians(camera_angle)

        vertical_fov_rad = 2 * math.atan(math.tan(horizontal_fov_rad / 2) / aspect_ratio)

        vertical_fov = math.degrees(vertical_fov_rad)

        mesafe = [0, 0]
        x=height/math.cos(camera_angle/57.3)
        #x = height / math.cos(camera_angle_rad)
        L=2*x*math.sin(horizontal_fov/114.6)
        #L = 2 * x * math.sin(horizontal_fov_rad / 2)
        m = (pixelheight - pos[1]) / pixelheight * L
        n = pos[1] / pixelheight * L
        h0 = x * math.cos(horizontal_fov/114.6)
        #h0 = x * math.cos(horizontal_fov_rad / 2)
        angle = math.asin(L / (2 * x)) - math.atan((n - m) / (2 * h0))
        mesafe[0]= height * math.tan(camera_angle/57.3 + angle)
        #mesafe[0] = height * math.tan(camera_angle_rad + angle)

        v = height / math.cos(camera_angle/57.3 + angle)
        #v = height / math.cos(camera_angle_rad + angle)
        p = v * math.tan(vertical_fov/114.6)
        #p = v * math.tan(vertical_fov_rad / 2)
        mesafe[1] = ((pixelwidth/2) - pos[0]) / (pixelwidth/2) * p

        
        return mesafe

    def calculate_distance(self, target_center):

        height = self.vehicle.altitude
        print("height:", height)
        position = target_center
        fov = 63
        camera_angle = 30

        relative_distance = self.hesap(height, position, fov, camera_angle)
        distance = math.sqrt(relative_distance[0] ** 2 + relative_distance[1] ** 2)

        #distance = 5

        print("distance:", distance)
        return distance

    def calculate_center(self, id,bbox):
        x_center = (bbox[0] + bbox[2]) / 2
        if id ==0:
            y_center = bbox[3]
        else:
            y_center = (bbox[1] + bbox[3]) / 2
        return (x_center, y_center)

    def get_point_at_distance(self, d, target_center, R=6371):
        """
        lat: initial latitude, in degrees
        lon: initial longitude, in degrees
        d: target distance from initial
        bearing: (true) heading in degrees
        R: optional radius of sphere, defaults to mean radius of earth

        Returns new lat/lon coordinate {d}km from initial, in degrees
        """
        
        lat1 = math.radians(self.vehicle.latitude)
        lon1 = math.radians(self.vehicle.longitude)
        target_offset = ((target_center[0] / 640) * 63) - 31.5

        #if self.vehicle.gimbal_flag == 0:
        #a = math.radians(self.vehicle.heading) + self.vehicle.gimbal_yaw_vehicle
        a = math.radians((self.vehicle.heading+target_offset)%360)
        
        print("target_angle:", math.degrees(a))
        #elif self.gimbal_flag == 1:
        #    a = self.vehicle.gimbal_yaw_earth

    
        

        lat2 = math.asin(math.sin(lat1) * math.cos(d / R) + math.cos(lat1) * math.sin(d / R) * math.cos(a))
        lon2 = lon1 + math.atan2(
            math.sin(a) * math.sin(d / R) * math.cos(lat1),
            math.cos(d / R) - math.sin(lat1) * math.sin(lat2)
        )
        return math.degrees(lat2), math.degrees(lon2)

    def track(self):
        print("Tracking started")
        while True:

            target = self.shared.get_target()
            if target is not None:
                target_center = self.calculate_center(target.id,target.bbox)
                

                distance = (self.calculate_distance(target_center) - 2)/1000

                #lat, lon = self.vehicle.get_point_at_distance(distance)
                lat, lon = self.get_point_at_distance(distance, target_center)

                #self.shared.set_roi(lat, lon, self.vehicle.altitude)
                print("destlat:", lat)
                print("destlon:", lon)
                
                self.vehicle.move_to(lat, lon)

                if self.shared.mission == "abort" or self.shared.mission == "land":
                    break

            else:
                pass
                #print("Target lost")
                #break

            time.sleep(.1)
