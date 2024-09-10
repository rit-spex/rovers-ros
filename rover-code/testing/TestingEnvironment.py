#!/usr/bin/env python3
"""
File: TestingEnvironment.py

Description: A testing environment to use for subsystem integration testing

Author: Ryan Barry
Date Created: September 15, 2023
"""
import rclpy
from hardware.RoverConstants import *
from rclpy.node import Node
from std_msgs.msg import Int32


class TestingEnvironment(Node):
    def __init__(self, rover):
        super().__init__( "testing_environment")
        self.rover = rover
        self.operating_mode_pub = self.create_publisher(Int32, "status_operating_mode_topic", 10)

        self.set_operating_mode(TEST_MODE) # Uncomment this line to use testing environment

    def run(self):
        print("WE'RE TESTING")
        pass

    def set_operating_mode(self, mode):
        msg = Int32()
        msg.data = mode
        self.operating_mode_pub.publish(msg)
