#!/usr/bin/env python3
"""
File: Humidity.py

This module contains a class enabling the 
operation of a Humidity Sensor

Author: Ryan Barry
Date Created: October 6, 2023
"""

import rclpy
from rclpy.node import Node


class Humidity(Node):
    def __init__(self):
        super().__init__( "Science_Humidity")
