#!/usr/bin/env python3
"""
File: LipidDetector.py

Description: This module contains the LipidDetector class, which gives the rover the ability to detect and publish the presense of lipids

Author: Ryan Barry
Date Created: October 8, 2023
"""
import rclpy  # Importing the rclpy module for ROS 2 Python client library
from rclpy.node import Node  # Importing the Node class from rclpy.node module
from hardware.RoverConstants import LIPID_DETECTED, LIPID_NOT_DETECTED
from std_msgs.msg import Int32  # Importing the Int32 message type from the std_msgs package


class LipidDetector(Node):
    def __init__(self):
        super().__init__( "science_lipid_detector")
        self.lipid_publisher = self.create_publisher(
            Int32, "science_lipid_detection", 10
        )  # Creating a publisher for lipid detection messages

        self.lipid_detection = LIPID_NOT_DETECTED


    def publish(self):  # Method for publishing lipid detection status
        lipid_detection_msg = Int32()  # Creating an instance of the Int32 message type
        lipid_detection_msg.data = (
            self.lipid_detection
        )  # Setting the data field of the message to the value of lipid_detection
        self.lipid_publisher.publish(lipid_detection_msg)  # Publishing the lipid detection message
        rclpy.spin_once(self)