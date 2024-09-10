#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hardware.RoverConstants import *
from std_msgs.msg import Float32, Int32


class Communications(Node):
    def __init__(self):
        super().__init__( "communications")

        # Initialize class variables
        self.waypoint_status = WAYPOINT_INACTIVE
        self.comm_link_status = NOT_CONNECTED
        self.operating_mode = DRIVER_CONTROL_MODE
        self.temperature = 0
        self.battery_voltage = 0
        self.battery_current = 0

        # Create publishers
        self.operating_mode_pub = self.create_publisher(Int32, "operating_mode_topic", 10)

        # Create subscribers
        self.waypoint_status_subscriber = self.create_subscription(
            Int32, "waypoint_status", self.waypoint_status_callback, 10
        )
        self.temperature_subscriber = self.create_subscription(
            Float32, "temperature", self.temperature_callback, 10
        )
        self.battery_voltage_subscriber = self.create_subscription(
            Float32, "battery_voltage", self.battery_voltage_callback, 10
        )
        self.battery_current_subscriber = self.create_subscription(
            Float32, "battery_current", self.battery_current_callback, 10
        )

    def waypoint_status_callback(self, msg):
        self.waypoint_status = msg.data

    def temperature_callback(self, msg):
        self.temperature = msg.data

    def battery_voltage_callback(self, msg):
        self.battery_voltage = msg.data

    def battery_current_callback(self, msg):
        self.battery_current = msg.data

    def update_operating_mode(self, mode=DRIVER_CONTROL_MODE):
        self.operating_mode = mode
        self.operating_mode_pub.publish(self.operating_mode)
