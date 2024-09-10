#!/usr/bin/env python3
"""
File: Camera.py

Description: This file contains the definition of the Camera class, 
which represents a ROS2 node for capturing and publishing camera images. 
It utilizes the OpenCV library to capture frames from a specified camera 
index and converts them to ROS2 Image messages for publication.

Author: Ryan Barry
Date Created: July 16, 2023

Dependencies:

    cv2: OpenCV library for capturing frames from the camera
    rclpy: ROS2 Python client library
    rclpy.node: Node class for creating ROS2 nodes
    sensor_msgs.msg.Image: ROS2 message type for representing images
    cv_bridge: Bridge library for converting between OpenCV and ROS2 image formats

"""


import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Camera(Node):
    def __init__(self, name, camera_index):
        self.__name = name
        self.__camera_index = camera_index
        
        super().__init__(f"cameras_{self.__name}_node")
        self.publisher = self.create_publisher(Image, f"cameras_{self.__name}_topic", 10)
        self.timer = self.create_timer(1.0 / 30, self.capture)  # Adjust the capture rate as needed
        self.cv_bridge = CvBridge()

    def capture(self, display=False):
        cv2.destroyWindow(f"{self.__name} Stream")
        # Capture the frame from the specified camera index using OpenCV
        cap = cv2.VideoCapture(self.__camera_index)
        ret, frame = cap.read()
        cap.release()

        # Convert the OpenCV frame to a ROS2 Image message
        if ret:
            if display:
                cv2.imshow(f"{self.__name} Stream", frame)
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the image message to the topic
            self.publisher.publish(image_msg)
