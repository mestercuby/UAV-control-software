#!/usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.
import threading

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rclpy for the subscriber
import rclpy
from rclpy.node import Node
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import os

# Instantiate CvBridge
bridge = CvBridge()


class Image_Subscriber(Node):

    def __init__(self, communication):
        super().__init__('minimal_subscriber')
        self.communication = communication

        print("Listener is started!")
        self.i = 0
        # Create a folder to save images
        self.path = "mission1"
        try:
            os.mkdir(self.path)
        except OSError:
            print(f"Creation of the directory {self.path} failed")

        image_topic = "/world/skywalker_runway/model/skywalker_x8_quad/model/gimbal/link/tilt_link/sensor/camera/image"
        # image_topic = "/world/alti_runway/model/alti_transition_quad/model/gimbal/link/tilt_link/sensor/camera/image"
        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            #print("Image is received!")
            # Save your OpenCV2 image as a jpg
            if self.communication is None:
                cv2.imshow("camera", cv2_img)
            else:
                self.communication.update_camera_image(cv2_img)

            # cv2.imwrite(f"{self.path}/camera_image{self.i}.jpg", cv2_img)
            # self.i = self.i + 1
        cv2.waitKey(1)


class ImageSubscriberThread(threading.Thread):
    def __init__(self, communication):
        super().__init__()
        self.communication = communication

    def run(self):
        rclpy.init()
        minimal_subscriber = Image_Subscriber(self.communication)
        rclpy.spin(minimal_subscriber)
        print("Listener is stopped!")
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


def start_listener(communication):
    rclpy.init()
    minimal_subscriber = Image_Subscriber(communication)
    rclpy.spin(minimal_subscriber)
    print("Listener is stopped!")
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    start_listener(None)
