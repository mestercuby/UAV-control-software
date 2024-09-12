import time

import numpy


class Communication:
    """
    This class is used to store the shared data between the main program and the other modules.
    """

    def __init__(self, fps):
        self.last_update_time = None
        self.error_msg = ""
        self.fps = fps

        # CV to server
        self.detections = None
        self.frame = None

        # CV to otonomi
        self.obj_detected = False  ### GÖZLEM İÇİN
        self.location = (-1, -1)

        # server to main
        self.mission = None
        self.argument = None

        # otnomi to server
        self.mission_finished = True
        # otonomi to CV
        self.lat = None
        self.long = None
        self.lidar_height = None
        self.uav_tilt = None
        self.gimbal_tilt = None
        # otonomi internal
        self.wanted_height = None
        self.homep = None
        # server to CV
        self.track_target = None

        # image from simulation
        self.camera_image = numpy.zeros((480, 640, 3), numpy.uint8)

        self.track_mission = False

    def update_camera_image(self, image):
        self.camera_image = numpy.copy(image)

    def update_detections(self, new_detections, new_frame):
        self.detections = new_detections
        self.frame = new_frame
        self.last_update_time = time.time()

    def get_detections(self):
        return self.detections, self.frame, self.last_update_time

    def update_CV_readings(self, new_lat, new_long, new_lidar_height, new_uav_tilt, new_gimbal_tilt):
        self.lat = new_lat
        self.long = new_long
        self.lidar_height = new_lidar_height
        self.uav_tilt = new_uav_tilt
        self.gimbal_tilt = new_gimbal_tilt

    def get_CV_readings(self):
        return self.lat, self.long, self.lidar_height, self.uav_tilt, self.gimbal_tilt

    def update_mission(self, new_message):
        command = new_message.split(' ')[0]
        if command=="track":
            argument = new_message.split(' ')[1] 
            self.argument = int(argument)
        else:
            self.argument = None
        self.mission = command
    
    def get_mission(self):
        return self.mission, self.argument

    def update_error_msg(self, error_msg):
        self.error_msg = error_msg

    def get_error_msg(self):
        return self.error_msg
    
    def update_target(self, target):
        self.track_target = target