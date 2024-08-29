#!/usr/bin/env python3
"""
File: Camera.py

Description: This file contains the definition of the Camera class, 
which represents a ROS node for capturing and publishing camera images. 
It utilizes the OpenCV library to capture frames from a specified camera 
index and converts them to ROS Image messages for publication.

Author: Ryan Barry
Date Created: July 16, 2023

Dependencies:

    cv2: OpenCV library for capturing frames from the camera
    sensor_msgs.msg.Image: ROS message type for representing images
    cv_bridge: Bridge library for converting between OpenCV and ROS image formats

"""


import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Camera:
    def __init__(self, name, camera_index):
        self.__name = name
        self.__camera_index = camera_index
        
        rospy.init_node(f"cameras/{self.__name}_node")
        self.publisher = rospy.Publisher(f"cameras/{self.__name}_topic", Image, queue_size=10)
        self.timer = rospy.Timer(1.0 / 30, self.capture)  # Adjust the capture rate as needed
        self.cv_bridge = CvBridge()

    def capture(self):
        # Capture the frame from the specified camera index using OpenCV
        cap = cv2.VideoCapture(self.__camera_index)
        ret, frame = cap.read()
        cap.release()

        # Convert the OpenCV frame to a ROS2 Image message
        if ret:
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the image message to the topic
            self.publisher.publish(image_msg)
        else:
            rospy.logwarn(f"Failed to capture frame from {self.__name} camera.")

if __name__ == "__main__":
    camera = Camera(rospy.get_param('~name'), rospy.get_param('~camera_index'))
