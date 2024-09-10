#!/usr/bin/env python3
""" 
File: Health.py 
 
Description: This file contains the implementation of a Health class that publishes 
temperature, battery voltage, and battery current data using ROS 2 (Robot Operating System 2). 
 
Author: Ryan Barry 
Date Created: August 11, 2023
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Health(Node):
    def __init__(self):
        super().__init__( "health_node")
        self.temperature_pub = self.create_publisher(Float32, "health/temperature", 10)
        self.voltage_pub = self.create_publisher(Float32, "health/battery_voltage", 10)
        self.current_pub = self.create_publisher(Float32, "health/battery_current", 10)

        self.temp = 0
        self.voltage = 0
        self.current = 0

    def update(self):
        self.temperature_pub.publish(self.temp)
        self.voltage_pub.publish(self.voltage)
        self.current_pub.publish(self.current)
