#!/usr/bin/env python3
"""
File: ArmRobot.py

Description:This module defines the ArmRobot class, which represents an arm robot with multiple joints. 
It provides functionality to add frames to the arm, move individual joints, update the Denavit-Hartenberg 
(DH) table, and perform forward and inverse kinematics calculations.


Author: Ryan Barry
Date Created: October 5, 2023
"""


from ArmRobotKinematics import PRISMATIC, REVOLUTE, FIXED_PRISMATIC, FIXED_REVOLUTE, ArmRobotKinematics
from Gripper import Gripper
from math import pi
import rospy
from geometry_msgs.msg import Twist

class ArmRobot(ArmRobotKinematics):
    def __init__(self):
        super().__init__()
        '''
        Additional initialization code specific to the arm robot:
        This is where you will define the configuration of your robot with the addFrame method

        The addFrame method expects the following parameters:
            joint_type: PRISMATIC or REVOLUTE
            theta_fix: The fixed angle (rad) of rotation about the z axis for a revolute joint. Default is 0.
            d: The distance (m) of translation along the z axis for a prismatic joint. Default is 0.
            a: The length (m) of the frame.
            alpha_fix: The fixed angle (rad) of rotation about the x axis. Default is 0.
        
        Example Usage:
            self.link1 = self.addFrame(joint_type=REVOLUTE, theta_fix=pi, a=LINK1_LENGTH)
            self.link2 = self.addFrame(joint_type=REVOLUTE, a=LINK2_LENGTH)
            self.link3 = self.addFrame(joint_type=REVOLUTE, a=WRIST_LENGTH)
            self.link4 = self.addFrame(joint_type=PRISMATIC, d=LINK4_LENGTH)
        '''
        rospy.init_node('arm_robot')
        rospy.Subscriber("/arm/cmd_vel", Twist, self.velocity_callback)
        self.velo = Twist()
        self.gripper = Gripper()

        self.run()
    
    def velocity_callback(self, msg):
        self.velo.linear = msg.linear
        self.velo.angular = msg.angular

        
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    arm_robot = ArmRobot()