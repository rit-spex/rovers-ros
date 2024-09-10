""" 
File: StatusLEDs.py 
 
Description: This file contains the implementation of three LED classes: 
CommLinkLED, OperatingModeLED, and WaypointLED. 
These classes inherit from the LED class and provide methods for updating 
the different Status LEDs used in the rover. 
 
Author: Ryan Barry
Date Created: August 16, 2023 
"""
from hardware.peripherals.LED import LED
from hardware.RoverConstants import *


class CommLinkLED(LED):
    def __init__(self, comm_link_status):
        self.comm_link_status = comm_link_status

    def update(self):
        if self.comm_link_status == CONNECTED:
            pass
        elif self.comm_link_status == NOT_CONNECTED:
            pass


class OperatingModeLED(LED):
    def __init__(self, operating_mode):
        self.operating_mode = operating_mode

    def update(self):
        if self.operating_mode == DRIVER_CONTROL_MODE:
            self.set_color(BLUE)
        elif self.operating_mode == AUTONOMOUS_MODE:
            self.set_color(RED)


class WaypointLED(LED):
    def __init__(self, waypoint_status):
        self.waypoint_status = waypoint_status

    def update(self):
        if self.waypoint_status == WAYPOINT_COMPLETE:
            self.set_color(GREEN)
