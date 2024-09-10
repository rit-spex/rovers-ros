#!/usr/bin/env python3
"""
File: DNADetector.py

Description: This module contains the DNADetector class, which gives the rover the ability to detect and publish the presense of DNA

Author: Ryan Barry
Date Created: October 8, 2023
"""
import rclpy  # Importing the rclpy module for ROS 2 Python client library
from rclpy.node import Node  # Importing the Node class from rclpy.node module
from hardware.RoverConstants import DNA_DETECTED, DNA_NOT_DETECTED
from std_msgs.msg import Int32  # Importing the Int32 message type from the std_msgs package


class DNADetector(Node):
    def __init__(self):
        super().__init__( "science_dna_detector")
        self.dna_publisher = self.create_publisher(
            Int32, "science_dna_detection", 10
        )  # Creating a publisher for DNA detection messages

        self.dna_detection = DNA_NOT_DETECTED

    def publish(self):  # Method for publishing DNA detection status
        dna_detection_msg = Int32()  # Creating an instance of the Int32 message type
        dna_detection_msg.data = (
            self.dna_detection
        )  # Setting the data field of the message to the value of dna_detection
        self.dna_publisher.publish(dna_detection_msg)  # Publishing the DNA detection message
