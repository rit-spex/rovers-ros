#!/usr/bin/env python3
"""
File: GPS.py

This module contains a class enabling the 
operation of a GPS device

Author: Ryan Barry
Date Created: August 27, 2023
"""

import rclpy
from rclpy.node import Node


class GPS(Node):
    def __init__(self):
        super().__init__( "GPS")
