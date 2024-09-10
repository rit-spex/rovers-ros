#!/usr/bin/env python3
"""
File: Fluorometer.py

This module contains a class enabling the 
operation of a Fluorometry sensor

Author: Ryan Barry
Date Created: October 6, 2023
"""

import rclpy
from rclpy.node import Node


class Fluorometer(Node):
    def __init__(self):
        super().__init__( "Science_Fluorometer")
