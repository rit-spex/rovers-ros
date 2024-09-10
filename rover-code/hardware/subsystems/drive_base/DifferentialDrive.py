"""
File: DifferentialDrive.py

Description: This file contains the implementation of a differential drive class for controlling a rover's movement. 
It includes methods for forward and inverse kinematics calculations.

Author: Ryan Barry
Date Created: August 12, 2023
"""

from math import cos, sin, sqrt

import numpy as np
from hardware.RoverConstants import BASE_WIDTH, WHEEL_RADIUS


class DifferentialDrive:
    def __init__(self, wheel_radius=WHEEL_RADIUS, wheel_base=BASE_WIDTH):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.q_dot = np.zeros(5, 1)
        self.target_velocity = np.zeros(4, 1)
        self.target_left_velocity = 0
        self.target_right_velocity = 0
        self.update_velocities()
        self.phi = 0

    def update_q_dot(self):
        self.q_dot = np.array(
            [self.phi_dot, self.x_dot, self.y_dot, self.thetaL_dot, self.thetaR_dot]
        ).reshape((5, 1))

    def update_velocities(self):
        self.phi_dot = self.q_dot[0]  # Angular Velocity
        self.x_dot = self.q_dot[1]  # X Velocity
        self.y_dot = self.q_dot[2]  # Y Velocicity
        self.thetaL_dot = self.q_dot[3]  # Left Wheel Velocity
        self.thetaR_dot = self.q_dot[4]  # Right Wheel Velocity

    def update_jacobians(self):
        r = self.wheel_radius
        d = self.wheel_base
        phi = self.phi

        # Calculate the Jacobian Matrix
        self.J = np.array(
            [
                [-r / d, -r / d],
                [r / 2 * cos(phi), r / 2 * cos(phi)],
                [r / 2 * sin(phi), r / 2 * sin(phi)],
                [1, 0],
                [0, 1],
            ]
        )
        self.J_inv = np.linalg.pinv(self.J)  # Calculate the pseudo-inverse of the Jacobian matrix

        self.C = np.array([[self.left_wheel_velocity], [self.right_wheel_velocity]])

    def forward_kinematics(self):
        self.update_jacobians()
        self.q_dot = np.dot(self.J, self.C)
        self.update_velocities()

    def inverse_kinematics(self):
        self.update_jacobians()

        wheel_velocities = np.dot(self.J_inv, self.target_velocity)

        self.thetaL_dot = wheel_velocities[0, 0]
        self.thetaR_dot = wheel_velocities[1, 0]

        self.update_q_dot()

        self.target_left_velocity = self.wheel_radius * self.thetaL_dot
        self.target_right_velocity = self.wheel_radius * self.thetaR_dot
