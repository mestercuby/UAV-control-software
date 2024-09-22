import math

class PositionEstimator :
    def __init__(self, vehicle, shared, height, width, fov):
        self.vehicle = vehicle
        self.shared = shared

        self.camera_image_width = height
        self.camera_image_height = width
        self.horizontal_fov = fov

        self.aspect_ratio = float(self.camera_image_width) / self.camera_image_height
        self.horizontal_fov_rad = math.radians(self.horizontal_fov)
        self.vertical_fov_rad = 2 * math.atan(math.tan(self.horizontal_fov_rad / 2) / self.aspect_ratio)
        print("vertical_fov:", math.degrees(self.vertical_fov_rad))

    def hesap(self, pos):
        camera_angle = math.pi / 2 + self.vehicle.gimbal_pitch + self.vehicle.pitch - (self.vertical_fov_rad / 2)
        height = self.vehicle.altitude
        mesafe = [0, 0]
        print('pos', pos)
        # calculating angle for y distance of the target
        x = height / math.cos(camera_angle)
        
        L = 2 * x * math.sin(self.vertical_fov_rad / 2)
        m = (self.camera_image_height - pos[1]) / self.camera_image_height * L
        n = pos[1] / self.camera_image_height * L
        h0 = x * math.cos(self.vertical_fov_rad / 2)
        y_offset_angle = self.vertical_fov_rad/2 - math.atan((n - m) / (2 * h0))
        print("y_offset_angle:", math.degrees(y_offset_angle))
        y_angle = camera_angle + y_offset_angle
        mesafe[1] = height * math.tan(y_angle)

        
        # calculating angle for x distance of the target
        x_offset_angle= self.vehicle.roll - self.horizontal_fov_rad / 2
        v = height / math.cos(y_angle)
        Z= v / math.cos(x_offset_angle) 
        L2 = 2 * Z * math.sin(self.horizontal_fov_rad / 2)
        m2 = pos[0] / self.camera_image_width * L2
        n2 = (self.camera_image_width - pos[0]) / self.camera_image_width * L2
        h02 = Z * math.cos(self.horizontal_fov_rad / 2)
        x_angle = x_offset_angle + self.horizontal_fov_rad / 2 - math.atan((n2 - m2) / (2 * h02))

        mesafe[0] = v * math.tan(x_angle)

        return mesafe
    

    def calculate_distance(self, target_center):
        position = target_center

        relative_distance = self.hesap(position)
        distance = math.sqrt(relative_distance[1] ** 2 + relative_distance[0] ** 2)
        target_angle = math.degrees(math.atan2(relative_distance[0], relative_distance[1]))
        print("relative_distance:", relative_distance)
        return distance, target_angle

    def calculate_center(self, class_id, bbox):
        x_center = (bbox[0] + bbox[2]) / 2
        if class_id == 0:
            y_center = bbox[3]
        else:
            y_center = (bbox[1] + bbox[3]) / 2
        return x_center, y_center

    def get_point_at_distance(self, d, target_angle):
        """
        lat: initial latitude, in degrees
        lon: initial longitude, in degrees
        d: target distance from initial
        bearing: (true) heading in degrees


        Returns new lat/lon coordinate {d}km from initial, in degrees
        """
        R = 6371  # Earth radius in km
        #print("vehicle_lat:", self.vehicle.latitude)
        #print("vehicle_lon:", self.vehicle.longitude)
        lat1 = math.radians(self.vehicle.latitude)
        lon1 = math.radians(self.vehicle.longitude)
        
        a = math.radians((self.vehicle.heading + target_angle + math.degrees(self.vehicle.gimbal_yaw)) % 360)
        """
        print("self.vehicle.heading:", self.vehicle.heading)
        print("target_angle:", target_angle)
        print("self.vehicle.gimbal_yaw:", math.degrees(self.vehicle.gimbal_yaw))
        """

        lat2 = math.asin(math.sin(lat1) * math.cos(d / R) + math.cos(lat1) * math.sin(d / R) * math.cos(a))
        lon2 = lon1 + math.atan2(
            math.sin(a) * math.sin(d / R) * math.cos(lat1),
            math.cos(d / R) - math.sin(lat1) * math.sin(lat2)
        )
        return math.degrees(lat2), math.degrees(lon2)
    
    def get_position(self, target,track=False,distance_offset=0):
        target_center = self.calculate_center(target.det_class, target.bbox)
        if track:
            iterative_distance = 10
        else:
            iterative_distance, target_angle = self.calculate_distance(target_center)

        distance = (iterative_distance-distance_offset)/1000
        lat, lon = self.get_point_at_distance(distance, target_angle)

        return (lat, lon) ,iterative_distance