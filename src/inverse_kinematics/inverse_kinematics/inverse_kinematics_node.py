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
from std_msgs.msg import Bool, Float32, Int16
from enum import IntEnum

from constants.CommandCodes import CONSTANTS

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

    # Current angles
    __curr_th0: float
    __curr_th1: float
    __curr_th2: float
    __curr_th3: float
    __curr_th4: float

    # Target angles
    __target_th0: float
    __target_th1: float
    __target_th2: float
    __target_th3: float
    __target_th4: float

    # Space mouse parameters
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

    # Node parameters
    __base_angle_publisher: Publisher
    __shoulder_angle_publisher: Publisher
    __elbow_angle_publisher: Publisher
    __wrist_bend_angle_publisher: Publisher
    __wrist_twist_angle_publisher: Publisher

    # Space mouse
    # __device: Any
    # __state: dict[str, int]
    # __buttonz: list[bool]
    # __homing: bool

    def __init__(self):
        super().__init__("Arm_inverse_kinematics")

        # Arm initial parameters
        # self.__initial_angles = [math.pi, math.pi / 2, -math.pi / 2, 0.0, 0.0]
        # self.__iAs = self.__initial_angles
        # self.__j0, self.__j1, self.__j2, self.__j3 = calc_joint_positions(
        #     self.__iAs[0], self.__iAs[1], self.__iAs[2], self.__iAs[3], False
        # )
        # self.__th0, self.__th1, self.__th2, self.__th3, self.__th4 = self.__initial_angles

        # General parameters
        # self.__point = self.__j3
        self.__curr_th0 = float('nan')
        self.__curr_th1 = float('nan')
        self.__curr_th2 = float('nan')
        self.__curr_th3 = float('nan')
        self.__curr_th4 = float('nan')

        self.__trans_sens = 0.004
        self.__rotate_sens = 0.00005

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )
#topic = f"/BASESTATION/{message_name}/{signal_name}"
        self.create_subscription(
            msg_type=Int16,
            topic=f"/BASESTATION/spacemouse/{CONSTANTS.SPACEMOUSE.AXIS_X_STR}",
            callback=self.__on_x_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Int16,
            topic=f"/BASESTATION/spacemouse/{CONSTANTS.SPACEMOUSE.AXIS_Y_STR}",
            callback=self.__on_y_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Int16,
            topic=f"/BASESTATION/spacemouse/{CONSTANTS.SPACEMOUSE.AXIS_Z_STR}",
            callback=self.__on_z_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Int16,
            topic=f"/BASESTATION/spacemouse/{CONSTANTS.SPACEMOUSE.AXIS_RX_STR}",
            callback=self.__on_rx_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Int16,
            topic=f"/BASESTATION/spacemouse/{CONSTANTS.SPACEMOUSE.AXIS_RY_STR}",
            callback=self.__on_ry_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Int16,
            topic=f"/BASESTATION/spacemouse/{CONSTANTS.SPACEMOUSE.AXIS_RZ_STR}",
            callback=self.__on_rz_received,
            qos_profile=10,
        )

        self.create_subscription(Float32, "/ARM/BASE/CURR_ANGLE", self.__on_base_angle_received, 10)
        self.create_subscription(Float32, "/ARM/SHOULDER/CURR_ANGLE", self.__on_shoulder_angle_received, 10)
        self.create_subscription(Float32, "/ARM/ELBOW/CURR_ANGLE", self.__on_elbow_angle_received, 10)
        self.create_subscription(Float32, "/ARM/WRIST_BEND/CURR_ANGLE", self.__on_wrist_bend_angle_received, 10)
        self.create_subscription(Float32, "/ARM/WRIST_TWIST/CURR_ANGLE", self.__on_wrist_twist_angle_received, 10)

        # create publishers for the arm motor angles 
        self.__base_angle_publisher        = self.create_publisher(Float32, "/ARM/BASE/TARGET_ANGLE", 10)
        self.__shoulder_angle_publisher    = self.create_publisher(Float32, "/ARM/SHOULDER/TARGET_ANGLE", 10)
        self.__elbow_angle_publisher       = self.create_publisher(Float32, "/ARM/ELBOW/TARGET_ANGLE", 10)
        self.__wrist_bend_angle_publisher  = self.create_publisher(Float32, "/ARM/WRIST_BEND/TARGET_ANGLE", 10)
        self.__wrist_twist_angle_publisher = self.create_publisher(Float32, "/ARM/WRIST_TWIST/TARGET_ANGLE", 10)

        # self.create_subscription(
        #     msg_type=Bool,
        #     topic=f"/BASESTATION/{CONSTANTS.SPACEMOUSE.NAME}/{CONSTANTS.SPACEMOUSE.MODE_BUTTON}",
        #     callback=self.__on_mode_received,
        #     qos_profile=10,
        # )

        # self.create_subscription(
        #     msg_type=Bool,
        #     topic=f"/BASESTATION/{CONSTANTS.SPACEMOUSE.NAME}/{CONSTANTS.SPACEMOUSE.HOMING_BUTTON}",
        #     callback=self.__on_homing_received,
        #     qos_profile=10,
        # )

        # self.__device = setup_spacemouse()
        # self.__state = {"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0, "buttons": 0}
        # self.__buttonz = [False, False]
        self.__homing = False
        self.__mode = False

    def __on_estop_received(self, msg: Bool):
        if msg.data:
            self.__running = False
            self.get_logger().info("ESTOP received. Stopping arm controller.")
        else:
            self.get_logger().info("ESTOP cleared. Arm controller can be restarted.")

    def __on_x_received(self, msg: Float32):
        if msg.data != self.__x:
            self.__x = msg.data
            self.__calculate_angles()

    def __on_y_received(self, msg: Float32):
        if msg.data != self.__y:
            self.__y = msg.data
            self.__calculate_angles()

    def __on_z_received(self, msg: Float32):
        if msg.data != self.__z:
            self.__z = msg.data
            self.__calculate_angles()

    def __on_rx_received(self, msg: Float32):
        if msg.data != self.__rx:
            self.__rx = msg.data
            self.__calculate_angles()

    def __on_ry_received(self, msg: Float32):
        if msg.data != self.__ry:
            self.__ry = msg.data
            self.__calculate_angles()

    def __on_rz_received(self, msg: Float32):
        if msg.data != self.__rz:
            self.__rz = msg.data
            self.__calculate_angles()

    def __on_mode_received(self, msg: Bool):
        if msg.data != self.__mode:
            self.__mode = msg.data
            if msg.data:
                self.get_logger().info("Joint control mode enabled.")
            else:
                self.get_logger().info("Position control mode enabled.")
            self.__calculate_angles()

    def __on_homing_received(self, msg: Bool):
        if msg.data != self.__homing:
            self.__homing = msg.data
            if msg.data:
                self.get_logger().info("Homing initiated.")
            else:
                self.get_logger().info("Homing cleared.")
            self.__calculate_angles()

    def __on_base_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th0:
            self.__curr_th0 = msg.data
            self.__update_point_from_angles()
            self.__calculate_angles()
    
    def __on_shoulder_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th1:
            self.__curr_th1 = msg.data
            self.__update_point_from_angles()
            self.__calculate_angles()
    
    def __on_elbow_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th2:
            self.__curr_th2 = msg.data
            self.__update_point_from_angles()
            self.__calculate_angles()

    def __on_wrist_twist_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th3:
            self.__curr_th3 = msg.data
            self.__update_point_from_angles()
            self.__calculate_angles()

    def __on_wrist_bend_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th4:
            self.__curr_th4 = msg.data
            self.__update_point_from_angles()
            self.__calculate_angles()

    def __check_current_angles_valid(self) -> bool:
        return not any(math.isnan(angle) for angle in [self.__curr_th0, self.__curr_th1, self.__curr_th2, self.__curr_th3, self.__curr_th4])
        

    def __update_point_from_angles(self):
        if not self.__check_current_angles_valid():
            self.get_logger().warning("Current angles not fully initialized. Cannot update point.")
            return

        _, _, self.__point, _ = calc_joint_positions(
            self.__curr_th0, self.__curr_th1, self.__curr_th2, self.__curr_th3, False
        )

    def __calculate_angles(self):
        if not self.__check_current_angles_valid():
            self.get_logger().warning("Current angles not fully initialized. Cannot calculate angles.")
            return

        self.__target_th0 = self.__curr_th0
        self.__target_th1 = self.__curr_th1
        self.__target_th2 = self.__curr_th2
        self.__target_th3 = self.__curr_th3
        self.__target_th4 = self.__curr_th4

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
                self.__target_th0,
                self.__target_th1,
                self.__target_th2,
                self.__target_th3,
                self.__target_th4,
            ) = homingStep(
                self.__homing,
                self.__target_th0,
                self.__target_th1,
                self.__target_th2,
                self.__target_th3,
                self.__target_th4,
            )
        elif self.__mode:
            self.__target_th0 = wrap_to_pi(
                self.__target_th0 + self.__rx * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3, False
            )

            self.__target_th1 = wrap_to_pi(
                self.__target_th1 + self.__ry * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3, False
            )

            self.__target_th2 = wrap_to_pi(
                self.__target_th2 + self.__rz * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3, False
            )
        else:
            # Move point inside cube
            self.__point[0] += self.__x * self.__trans_sens
            self.__point[1] += self.__y * self.__trans_sens
            self.__point[2] += self.__z * self.__trans_sens

            # x rotates the gripper
            self.__target_th3 += self.__rx * self.__rotate_sens
            # z spins the gripper
            self.__target_th4 += self.__rz * self.__rotate_sens

            # Compute arm joints
            try:
                self.__th0, self.__th1, self.__th2, _ = inverse_kinematics(
                    self.__point,
                    [self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3],
                    self.__target_th4,
                )

            except Exception as e:
                print(e)

        # # Reset state variable
        # for key in self.__state:
        #     self.__state[key] = 0

        # Update stuff
        self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3, self.__target_th4 = (
            joint_limits(
                self.__target_th0,
                self.__target_th1,
                self.__target_th2,
                self.__target_th3,
                self.__target_th4,
                not self.__mode,
            )
        )

        self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3, self.__target_th4 = (
            collision_protection(
                self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3, self.__target_th4
            )
        )
        _, _, l2, _ = calc_joint_positions(
            self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3
        )
        self.__point = l2

        self.__base_angle_publisher.publish(Float32(data=self.__target_th0))
        self.__shoulder_angle_publisher.publish(Float32(data=self.__target_th1))
        self.__elbow_angle_publisher.publish(Float32(data=self.__target_th2))
        self.__wrist_twist_angle_publisher.publish(Float32(data=self.__target_th3))
        self.__wrist_bend_angle_publisher.publish(Float32(data=self.__target_th4))        
    def run(self):
        self.get_logger().info("starting inverse kinematics node...")
        rclpy.spin(self)
        self.get_logger().info("stopping inverse kinematics node...")

def main():
    rclpy.init()
    arm_controller = ArmController()
    arm_controller.run()

if __name__ == "__main__":
    main()
