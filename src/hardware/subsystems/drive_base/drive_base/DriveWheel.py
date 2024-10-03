#!/usr/bin/env python3
"""
File: DriveWheel.py

Description: This script contains the DriveWheel class, which inherits from Motor.
A drive wheel object can be instantiated within the drive base of the rover.
The class includes a function to calculate and publish its velocity to a topic.

Author: Ryan Barry
Date Created: August 12, 2023
"""

import rospy
from std_msgs.msg import Float32

from hardware.peripherals.Motor import Motor
from hardware.RoverConstants import *
from hardware.subsystems.drive_base.VelocityPublisher import VelocityPublisher


class DriveWheel(Motor):
    def __init__(self, name: str, pwm_pin: int = None):
        rospy.init_node(self, f"{name}_node")
        Motor.__init__(self, pwm_pin=pwm_pin)

        self.__name = name
        self.__wheel_num = WHEEL_NAMES.index(self.__name)

        rospy.Subscriber(f"velocity_topics/{self.__name}_velocity", Float32, self.velocity_callback)
        # self.velo_sub = self.create_subscription(
        #     Float32, f"velocity_topics_{self.__name}_velocity", self.velocity_callback, 10
        # )

        # ROS2 Publisher to publish the velocity of the wheel
        self.__velo_pub = rospy.Publisher(self.__name)

        self.__velocity = 0.0
        self.__target_velocity = 0.0
        self.__wheel_radius = WHEEL_RADIUS



    def velocity_callback(self, msg):
        print(f"msg: {msg}")
        print(f"data: {msg.data}")
        self.__target_velocity = msg.data

    def set_radius(self, radius):
        self.__wheel_radius = radius

    def calculate_velocity(self):
        # Insert code here to calculate the velocity of the wheel
        self.__velo_pub.publish_velocity(self.__velocity)

    def get_velocity(self):
        return self.__velocity
