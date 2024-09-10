#!/usr/bin/env python3
"""
File: IMU.py

This module contains a class enabling the 
operation of an inertial measurement unit

Author: Ryan Barry
Date Created: October 5, 2023
"""

import rclpy
from rclpy.node import Node


class IMU(Node):
    def __init__(self):
        super().__init__( "IMU")
