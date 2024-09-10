#!/usr/bin/env python3
"""
File: DriveWheel.py

Description: This script contains the DriveWheel class, which inherits from Motor.
A drive wheel object can be instantiated within the drive base of the rover.
The class includes a function to calculate and publish its velocity to a topic.

Author: Ryan Barry
Date Created: August 12, 2023
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from controllers.pid_controller import PIDController
from hardware.peripherals.Motor import Motor
from hardware.RoverConstants import *
from hardware.subsystems.drive_base.VelocityPublisher import VelocityPublisher


class DriveWheel(Motor, Node):
    def __init__(self, name: str, pwm_pin: int = None):
        Node.__init__(self, f"{name}_node")
        Motor.__init__(self, pwm_pin=pwm_pin)

        self.__name = name
        self.__wheel_num = WHEEL_NAMES.index(self.__name)

        # Ros2 Subscriber for the velocity of the wheel
        print(f"Creating a subscription with the name:\nvelocity_topics_{self.__name}_velocity\n{self.velocity_callback}\n...")
        self.velo_sub = self.create_subscription(
            Float32, f"velocity_topics_{self.__name}_velocity", self.velocity_callback, 10
        )

        # ROS2 Publisher to publish the velocity of the wheel
        self.__velo_pub = VelocityPublisher(self.__name)

        self.__velocity = 0.0
        self.__target_velocity = 0.0
        self.__wheel_radius = WHEEL_RADIUS
        self.__pid_controller = PIDController(
            kp=KP[self.__wheel_num], ki=KI[self.__wheel_num], kd=KD[self.__wheel_num]
        )
        if pwm_pin is not None:
            try:
                """
                The GPIO class has beeen fixed.
                The issue right now is sudo permissions to access the GPIO pins.
                Need to figure out how to get the program access to those. 
                Might involve vitis/vivado stuff idk, Josh may know more or it may be something that can be found online.
                
                """
                self.pwm_pin = self.select_pwm_pin(pwm_pin)
            except Exception as E:
                print(f"this shit donked out dude {E.args}")

        rclpy.spin(self)

        self.get_logger().info(f"{self.__name} DriveWheel initialized.")

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
