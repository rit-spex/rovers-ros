#!/usr/bin/env python3
"""
File: VelocityPublisher.py

Description: This module contains a publisher object to a velocity
topic.

Author: Ryan Barry
Date Created: Septermber 24, 2023
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class VelocityPublisher(Node):
    def __init__(self, name):
        self.name = name
        super().__init__(f"{self.name}_velocity_publisher")
        self.publisher_ = self.create_publisher(
            Float32, f"velocity_topics/{self.name}_velocity", 10
        )

    def publish_velocity(self, velocity):
        msg = Float32()
        msg.data = velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published {self.name} velocity: %f" % velocity)
