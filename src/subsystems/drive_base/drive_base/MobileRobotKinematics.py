"""
This module provides the `MobileRobotKinematics` class which calculates the kinematic properties of any fixed-wheel mobile robot

Default values for the `MobileRobotKinematics` class are defined at the top of the file. Changing these values to fit any robot
configuration will update the kinematics accordingly.

Requirements: numpy

Author: Ryan Barry
Date created: September 14, 2023
"""

from math import cos, pi, sin

import numpy as np
import rospkg
import sys
import rospy

rospack = rospkg.RosPack()
package_path = rospack.get_path('constants')
sys.path.append(package_path + '/src')

from RoverConstants import ALPHA, BASE_WIDTH, BETA, GAMMA, WHEEL_NAMES, WHEEL_RADIUS


class MobileRobotKinematics:
    def __init__(
        self,
        beta=BETA,
        wheel_names=WHEEL_NAMES,
        wheel_radius=WHEEL_RADIUS,
        L=BASE_WIDTH,
        alpha=ALPHA
    ):
        """
        Initializes the `MobileRobotKinematics` instance.
        :param beta: The angle between center of robot and wheel rotation axis for each wheel
        :param wheel_names: The names of wheels in the robot.
        :param wheel_radius: The radius of the wheel in meters.
        :param L: The distance from the center of the robot to the center of the wheels in meters.
        :param alpha: The angle offset from the robot's x-axis of each wheel
        """
        # Define variables
        self.__beta = beta
        self.__wheel_radius = wheel_radius
        self.__L = L
        self.__wheel_names = wheel_names
        self.__num_wheels = len(self.__wheel_names)
        self.__alpha = alpha
        self.__theta = 0 # Robot's angle relative to world frame
        self.__r_theta = self.update_heading(self.__theta)
        self.__J1_list = []
        self.__C1_list = []
        self.__J2 = np.eye(self.__num_wheels) * self.__wheel_radius

        for i, _ in enumerate(self.__wheel_names):
            self.__J1_list.append(
                np.array(
                    [
                        sin(self.__alpha[i] + self.__beta[i]),
                        -cos(self.__alpha[i] + self.__beta[i]),
                        -self.__L[i] * cos(self.__beta[i]),
                    ]
                )
            )
            self.__C1_list.append(
                np.array(
                    [
                        cos(self.__alpha[i] + self.__beta[i]),
                        sin(self.__alpha[i] + self.__beta[i]),
                        self.__L[i] * sin(self.__beta[i]),
                    ]
                )
            )
        
        self.__J1 = np.array(self.__J1_list)
        self.__C1 = np.array(self.__C1_list)
        self._zeta_dot = np.zeros((3, 1))
        self._phi = np.zeros((self.__num_wheels, 1))

    def calculate_robot_velocity(self, velocities: list):
        self._phi = np.array(velocities)
        return self.forward_kinematics()

    def calculate_wheel_velocities(self, velocity: list):
        self._zeta_dot = np.array(velocity)
        return self.inverse_kinematics()
    
    def update_heading(self, theta):
        self.__theta = theta
        self.__r_theta = np.array(
            [
                [cos(self.__theta), sin(self.__theta), 0],
                [-sin(self.__theta), cos(self.__theta), 0],
                [0, 0, 1],
            ]
        )

    # Calculates the current linear and angular velocities of the robot
    def forward_kinematics(self):
        """
        Calculates the vector of linear and angular velocities zeta_dot in the form
                    [  x_dot  ] # linear velocity in the x direction
        zeta_dot =  [  y_dot  ] # linear velocity in the y direction
                    [ phi_dot ] # angular velocity about the z axis
        """
        self._zeta_dot = np.linalg.inv(self.__r_theta) @ self.__J1 @ self.__J2 @ self._phi
        return self._zeta_dot

    # Calculates the inverse kinematics of the robot
    def inverse_kinematics(self):
        """
        Calculates the vector of wheel velocities necessary to achieve a targeted robot velocity
                [ wheel_1 ]
        phi =   [ wheel_2 ]
                [   ...   ]
                [ wheel_n ]
        """
        self.update_heading(self.__theta)
        self._phi = np.linalg.pinv(self.__J2) @ self.__J1 @ self.__r_theta @ self._zeta_dot
        return self._phi
