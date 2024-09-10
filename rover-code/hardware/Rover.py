#!/usr/bin/env python3
"""
File: Rover.py

Description:This file defines the "Rover" class, which represents 
the main hardware and functionalities of the rover. It acts as an 
interface between the rover's missions and its various subsystems. 
The class handles the initialization of the rover's status, peripherals, 
and subsystems, as well as updating the LEDs based on the rover's status.

Author: Ryan Barry
Date Created: July 16, 2023
"""


import rclpy

from communications.Communications import Communications
from ErrorHandler import ErrorHandler
from hardware.status.RoverStatus import RoverStatus
from hardware.status.StatusLEDs import CommLinkLED, OperatingModeLED, WaypointLED
from rclpy.node import Node
from hardware.RoverConstants import *
from hardware.RoverPinout import *
from hardware.sensors.GPS import GPS
from hardware.sensors.LiDAR import LiDAR
from hardware.subsystems.arm.ArmRobot import ArmRobot
from hardware.subsystems.cameras.FrontCamera import FrontCamera
from hardware.subsystems.drive_base.DriveBase import DriveBase
from hardware.subsystems.science_plate.SciencePlate import SciencePlate


class Rover(Node, ErrorHandler):
    def __init__(self):
        ErrorHandler.__init__(self)
        Node.__init__(self,"rover")

        # Initialize Status
        self.status = RoverStatus()
        self.__active_mission = None
        print("status")
        # Initialize Subsystems
        self.arm = ArmRobot()
        print("arm")
        self.science_plate = SciencePlate()
        print("science plate")
        self.drive_base = DriveBase(self.status.operating_mode)
        print("drive base")
        self.comms = Communications()
        print("comms")

        # Initialize LEDs
        self.operating_mode_LED = OperatingModeLED(self.status.operating_mode)
        self.comm_link_LED = CommLinkLED(self.status.comm_link_status)
        self.waypoint_LED = WaypointLED(self.status.waypoint_status)

        # Initialize Sensors
        self.lidar = LiDAR()
        self.gps = GPS()
        self.front_cam = FrontCamera()

    def get_mission(self):
        if self.__active_mission in missions:
            return self.__active_mission
        # self.log_error(MISSION_FAILURE)

    def set_mission(self, mission):
        self.__active_mission = mission

    def update_LEDs(self):
        self.operating_mode_LED.update()
        self.comm_link_LED.update()
        self.waypoint_LED.update()

    def run(self):
        rclpy.spin_once(self)  # Spin the node
        self.status.spin()

        self.status.destroy_node()
        self.arm.destroy_node()
