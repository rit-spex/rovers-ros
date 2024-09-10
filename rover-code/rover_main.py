#!/usr/bin/env python3
"""
File: rover_main.py

Description: This script serves as the main control module for a rover system. 
It utilizes the Robot Operating System (ROS) client library (rclpy) to interface 
with the rover's hardware components. The program instantiates the MissionControl class 
responsible for managing various missions that the rover can execute.

Author: Ryan Barry
Date Created: August 26, 2023
"""

import rclpy
from missions.MissionControl import MissionControl


def main(args=None):
    rclpy.init(args=args)
    mission_control = MissionControl()

    while rclpy.ok():
        mission_control.exec()
        rclpy.spin_once(mission_control.controller)

    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except:
        rclpy.shutdown()
