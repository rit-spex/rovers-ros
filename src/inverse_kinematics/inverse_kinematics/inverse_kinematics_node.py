import math
from linear_algebra import (
    calc_joint_positions,
    collision_protection,
    homingStep,
    inverse_kinematics,
    joint_limits,
)
from math_helpers import wrap_to_pi
import rclpy
# from space_mouse import (
#     read_spacemouse,
#     setup_spacemouse,
# )

# Typing
from rclpy.node import Node
from typing import Any
from numpy import dtype, ndarray

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Bool, Float32
from enum import IntEnum

"""
Controls
- No Buttons (Position Drive)
    - Tx : Translation of wrist along x-axis
    - Ty : Translation of wrist along y-axis
    - Tz : Translation of wrist along z-axis
    - Rx : Rotation of the gripper along the wrist axis
    - Rz : Rotation of the gripper along the lead-screw axis
- Left Button
    - Homing Function : Drives the arm to point straight up
- Right Button (Joint Drive)
    - Rx : Rotation of the elbow
    - Ry : Rotation of the shoulder
    - Rz : Rotation of the base
"""


class ArmController(Node):
    # Arm initial parameters
    __initial_angles: list[float]
    __iAs: list[float]
    __j0: ndarray[tuple[int], dtype[Any]]
    __j1: ndarray[tuple[int], dtype[Any]]
    __j2: ndarray[tuple[int], dtype[Any]]
    __j3: ndarray[tuple[int], dtype[Any]]
    __th0: float
    __th1: float
    __th2: float
    __th3: float
    __th4: float

    __x: float
    __y: float
    __z: float
    __rx: float
    __ry: float
    __rz: float
    __homing: bool
    __mode: bool # Buttonz


    # General parameters
    __point: ndarray[tuple[int], dtype[Any]]
    __trans_sens: float
    __rotate_sens: float

    # Space mouse
    # __device: Any
    # __state: dict[str, int]
    # __buttonz: list[bool]
    # __homing: bool

    def __init__(self):
        super().__init__("Arm_inverse_kinematics")

        # Arm initial parameters
        self.__initial_angles = [math.pi, math.pi / 2, -math.pi / 2, 0.0, 0.0]
        self.__iAs = self.__initial_angles
        self.__j0, self.__j1, self.__j2, self.__j3 = calc_joint_positions(
            self.__iAs[0], self.__iAs[1], self.__iAs[2], self.__iAs[3], False
        )
        self.__th0, self.__th1, self.__th2, self.__th3, self.__th4 = self.__initial_angles

        # General parameters
        self.__point = self.__j3
        self.__trans_sens = 0.004
        self.__rotate_sens = 0.00005

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        # # Space mouse
        # self.__device = setup_spacemouse()
        # self.__state = {"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0, "buttons": 0}
        # self.__buttonz = [False, False]
        # self.__homing = False

    def __on_estop_received(self, msg: Bool):
        if msg.data:
            self.__running = False
            self.get_logger().info("ESTOP received. Stopping arm controller.")
        else:
            self.get_logger().info("ESTOP cleared. Arm controller can be restarted.")

    def __on_mode_received(self, msg: Bool):
        self.__mode = msg.data



    def run(self):
            
            # Read mouse data
            # read_spacemouse(self.__device, self.__state)

            # if int(self.__state["buttons"]) == 1:
            #     self.__homing = True
            # if int(self.__state["buttons"]) == 2:
            # self.__buttonz[1] = not (self.__buttonz[1])

            # Check if anything is going on
            # if (not all(v == 0 for v in self.__state.values())) or self.__homing:
                # Okay well what is going on
        if self.__homing:
            (
                self.__homing,
                self.__th0,
                self.__th1,
                self.__th2,
                self.__th3,
                self.__th4,
            ) = homingStep(
                self.__homing,
                self.__th0,
                self.__th1,
                self.__th2,
                self.__th3,
                self.__th4,
            )
        elif self.__mode:
            self.__th2 = wrap_to_pi(
                self.__th2 + self.__rx * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__th0, self.__th1, self.__th2, self.__th3, False
            )

            self.__th1 = wrap_to_pi(
                self.__th1 + self.__ry * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__th0, self.__th1, self.__th2, self.__th3, False
            )

            self.__th0 = wrap_to_pi(
                self.__th0 - self.__rz * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__th0, self.__th1, self.__th2, self.__th3, False
            )
        else:
            # Move point inside cube
            self.__point[0] += self.__x * self.__trans_sens
            self.__point[1] += self.__y * self.__trans_sens
            self.__point[2] += self.__z * self.__trans_sens

            # x rotates the gripper
            self.__th3 += self.__rx * self.__rotate_sens
            # z spins the gripper
            self.__th4 += self.__rz * self.__rotate_sens

            # Compute arm joints
            try:
                self.__th0, self.__th1, self.__th2, _ = inverse_kinematics(
                    self.__point,
                    [self.__th0, self.__th1, self.__th2, self.__th3],
                    self.__th4,
                )

            except Exception as e:
                print(e)

        # # Reset state variable
        # for key in self.__state:
        #     self.__state[key] = 0

        # Update stuff
        self.__th0, self.__th1, self.__th2, self.__th3, self.__th4 = (
            joint_limits(
                self.__th0,
                self.__th1,
                self.__th2,
                self.__th3,
                self.__th4,
                not self.__mode,
            )
        )

        self.__th0, self.__th1, self.__th2, self.__th3, self.__th4 = (
            collision_protection(
                self.__th0, self.__th1, self.__th2, self.__th3, self.__th4
            )
        )
        _, _, l2, _ = calc_joint_positions(
            self.__th0, self.__th1, self.__th2, self.__th3
        )
        self.__point = l2

def main():
    rclpy.init()
    arm_controller = ArmController()
    arm_controller.run()

if __name__ == "__main__":
    main()
