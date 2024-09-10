#!/usr/bin/env python3
"""
File: RoverStatus.py

Description: This module defines the RoverStatus class, which represents the status of a rover in a ROS2 system. 
It provides functionality to subscribe to various topics and update the rover's operating mode, communication link 
status, and waypoint status based on received messages.

Author: Ryan Barry
Date created: June 26, 2023

Dependencies:

    rclpy: ROS2 Python client library
    rclpy.node: Node class for creating ROS2 nodes
    std_msgs.msg.Int32: ROS2 message type for representing integer values
    hardware.RoverConstants: Constants related to the rover's hardware configuration and states

"""
import rclpy
from hardware.RoverConstants import *
from rclpy.node import Node
from std_msgs.msg import Int32


class RoverStatus(Node):
    def __init__(self):
        super().__init__( "rover_status")
        self.operating_mode = DRIVER_CONTROL_MODE
        self.comm_link_status = NOT_CONNECTED
        self.waypoint_status = WAYPOINT_INACTIVE

        # Create subscribers
        self.__operating_mode_subscriber = self.create_subscription(
            Int32, "status/operating_mode_topic", self.operating_mode_callback, 10
        )
        self.__comm_link_status_subscriber = self.create_subscription(
            Int32, "status/comm_link_status_topic", self.comm_link_status_callback, 10
        )
        self.__waypoint_status_subscriber = self.create_subscription(
            Int32, "status/waypoint_status_topic", self.waypoint_status_callback, 10
        )
        

    def spin(self):
        # Start spinning the node
        rclpy.spin_once(self)

    def operating_mode_callback(self, msg):
        self.operating_mode = msg.data

    def comm_link_status_callback(self, msg):
        self.comm_link_status = msg.data

    def waypoint_status_callback(self, msg):
        self.waypoint_status = msg.data
