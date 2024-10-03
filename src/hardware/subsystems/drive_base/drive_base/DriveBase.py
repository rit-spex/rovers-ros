#!/usr/bin/env python3
"""
File: DriveBase.py

Description: This file contains the implementation of a DriveBase class that
extends the DifferentialDrive class. It provides methods for controlling the
movement of a rover using drive wheels.

Author: Ryan Barry
Date Created: August 12, 2023
"""

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import rospkg
import sys

rospack = rospkg.RosPack()
package_path = rospack.get_path('communications')
sys.path.append(package_path + '/src')
package_path = rospack.get_path('constants')
sys.path.append(package_path + '/src')



# from DriveWheel import DriveWheel
from MobileRobotKinematics import MobileRobotKinematics
from RoverConstants import WHEEL_NAMES
from RoverPinout import *
from CAN_Constants import TOPICS
from communications.msg import CAN_msg


key_checker = False
for key, value in TOPICS.items():
    if value["name"] == "TARGET_VELOCITY":
        TARGET_VELOCITY_INDEX = key
        if key_checker:
            break
        key_checker = True
    if value["name"] == "CURRENT_VELOCITY":
        CURRENT_VELOCITY_INDEX = key
        if key_checker:
            break
        key_checker = True


MAX_WHEEL_VEL = 32


class DriveBase(MobileRobotKinematics):
    def __init__(self):
        MobileRobotKinematics.__init__(self)

        rospy.init_node("drive_base_node")
        rospy.Subscriber(f"/CAN/RX/{TOPICS[CURRENT_VELOCITY_INDEX]['name']}", CAN_msg, self.current_velocity_callback)
        rospy.Subscriber("/drive_base/cmd_vel", Twist, self.target_velocity_callback)
        self.velo_pub = rospy.Publisher(f"/CAN/TX/{TOPICS[TARGET_VELOCITY_INDEX]['name']}", CAN_msg, queue_size=10)

        self.new_velocity = False


        # A dictionary of publishers for the targeted velocities of each wheel
        self.wheels = {}

        for i in range(len(WHEEL_NAMES)):
            name = WHEEL_NAMES[i]
            pwm_pin = WHEEL_PINS[f"{name}_pwm"]

            # Add a tuple of VelocityPublisher and DriveWheel objects to the
            # self.wheels dictionary for each wheel name
            velo_pub = rospy.Publisher(f"velocity/{name}_velocity", Float32, queue_size=10)
            # wheel = DriveWheel(name=name, pwm_pin=pwm_pin)
            self.wheels[name] = (velo_pub, name)

        self.run()


    def current_velocity_callback(self, msg):
        # Process
        pass

    def target_velocity_callback(self, msg: Twist):
        target_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.calculate_wheel_velocities(target_vel)

        self.new_velocity = True


    def encoder(self, velo):
        #do our scaling here
        scaler = min(abs(velo / MAX_WHEEL_VEL), 1)
        return int(255*scaler)

    def run(self):
        while not rospy.is_shutdown():
            if self.new_velocity:
                velo_can_msg = CAN_msg()
                velo_can_msg.buf = bytearray(velo_can_msg.buf)
                velo_can_msg.id = TOPICS[TARGET_VELOCITY_INDEX]['id']
                phi_list = self._phi.T.tolist()

                for i, velo in enumerate(phi_list):
                    velo_can_msg.buf[i] = self.encoder(velo)
                    if velo >= 0:
                        velo_can_msg.buf[6] &= ~(1 << i)
                    else:
                        velo_can_msg.buf[6] |= (1 << i)


                self.velo_pub.publish(velo_can_msg)
                self.new_velocity = False

if __name__ == "__main__":
    drive_base = DriveBase()
