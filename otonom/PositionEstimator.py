import math
import time

class PositionEstimator :
    def __init__(self, vehicle, shared):
        self.vehicle = vehicle
        self.shared = shared

        self.camera_image_width = 640
        self.camera_image_height = 480
        self.horizontal_fov = 63

    def hesap(self, pos):
        aspect_ratio = float(self.camera_image_width) / self.camera_image_height
        horizontal_fov_rad = math.radians(self.horizontal_fov)
        vertical_fov_rad = 2 * math.atan(math.tan(horizontal_fov_rad / 2) / aspect_ratio)

        camera_angle = math.pi / 2 - (self.vehicle.gimbal_pitch-self.vehicle.pitch + vertical_fov_rad / 2)
        height = self.vehicle.altitude
        mesafe = [0, 0]


        # calculating angle for x distance of the target
        x = height / math.cos(camera_angle)
        L = 2 * x * math.sin(vertical_fov_rad / 2)
        m = (self.camera_image_height - pos[1]) / self.camera_image_height * L
        n = pos[1] / self.camera_image_height * L
        h0 = x * math.cos(vertical_fov_rad / 2)
        x_offset_angle = vertical_fov_rad/2 - math.atan((n - m) / (2 * h0))
        x_angle = camera_angle + x_offset_angle

        mesafe[0] = height * math.tan(x_angle)

        
        # calculating angle for y distance of the target
        y_offset_angle= self.vehicle.roll - horizontal_fov_rad / 2
        v = height / math.cos(x_angle)
        Z= v / math.cos(y_offset_angle) 
        L2 = 2 * Z * math.sin(horizontal_fov_rad / 2)
        m2 = pos[0] / self.camera_image_width * L2
        n2 = (self.camera_image_width - pos[0]) / self.camera_image_width * L2
        h02 = Z * math.cos(horizontal_fov_rad / 2)
        y_angle = y_offset_angle + horizontal_fov_rad / 2 - math.atan((n2 - m2) / (2 * h02))

        mesafe[1] = v * math.tan(y_angle)

        return mesafe

    def calculate_distance(self, target_center):
        position = target_center

        relative_distance = self.hesap(position)
        distance = math.sqrt(relative_distance[0] ** 2 + relative_distance[1] ** 2)

        print("distance:", distance)
        return distance

    def calculate_center(self, class_id, bbox):
        x_center = (bbox[0] + bbox[2]) / 2
        if class_id == 0:
            y_center = bbox[3]
        else:
            y_center = (bbox[1] + bbox[3]) / 2
        return x_center, y_center

    def get_point_at_distance(self, d, target_center):
        """
        lat: initial latitude, in degrees
        lon: initial longitude, in degrees
        d: target distance from initial
        bearing: (true) heading in degrees


        Returns new lat/lon coordinate {d}km from initial, in degrees
        """
        R = 6371  # Earth radius in km
        print("vehicle_lat:", self.vehicle.latitude)
        print("vehicle_lon:", self.vehicle.longitude)
        lat1 = math.radians(self.vehicle.latitude)
        lon1 = math.radians(self.vehicle.longitude)
        target_offset = ((target_center[0] / self.camera_image_width) * self.horizontal_fov) - self.horizontal_fov/2
        print("degree:", target_offset)
        a = math.radians((self.vehicle.heading + target_offset) % 360)

        print("target_angle:", math.degrees(a))

        lat2 = math.asin(math.sin(lat1) * math.cos(d / R) + math.cos(lat1) * math.sin(d / R) * math.cos(a))
        lon2 = lon1 + math.atan2(
            math.sin(a) * math.sin(d / R) * math.cos(lat1),
            math.cos(d / R) - math.sin(lat1) * math.sin(lat2)
        )
        return math.degrees(lat2), math.degrees(lon2)
    
    def get_position(self, target,track=False,distance_offset=2):
        target_center = self.calculate_center(target.det_class, target.bbox)
        if track:
            iterative_distance = 10
        else:
            iterative_distance = self.calculate_distance(target_center)

        distance = (iterative_distance-distance_offset)/100
        lat, lon = self.get_point_at_distance(distance, target_center)

        return lat, lon