import math
from linear_algebra import (
    calc_joint_positions,
    collision_protection,
    homingStep,
    inverse_kinematics,
    joint_limits,
)
from math_helpers import wrap_to_pi, wrap_to_minus_90
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
from std_msgs.msg import Bool, Float32, Int16, UInt16
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

UPDATE_RATE_SEC = 0.5  # seconds


class ARM_MODES(IntEnum):
    RAW_CONTROL = 0
    POINT_CONTROL = 1
    GRIPPER_CONTROL = 2


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
    __curr_th5: float

    # Target angles
    __target_th0: float
    __target_th1: float
    __target_th2: float
    __target_th3: float
    __target_th4: float
    __target_th5: float

    # Space mouse parameters
    __x: float
    __y: float
    __z: float
    __rx: float
    __ry: float
    __rz: float
    __homing: bool
    __homing_toggled: bool
    __mode: ARM_MODES  # Buttonz
    __mode_toggled: bool

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
    __gripper_angle_publisher: Publisher

    # Space mouse
    # __device: Any
    # __state: dict[str, int]
    # __buttonz: list[bool]
    # __homing: bool

    def __init__(self):
        super().__init__("Arm_inverse_kinematics")

        self.__curr_th0 = float("nan")
        self.__curr_th1 = float("nan")
        self.__curr_th2 = float("nan")
        self.__curr_th3 = float("nan")
        self.__curr_th4 = float("nan")
        self.__curr_th5 = float("nan")

        self.__x = 0
        self.__y = 0
        self.__z = 0
        self.__rx = 0
        self.__ry = 0
        self.__rz = 0

        self.__trans_sens = 0.004
        self.__rotate_sens = 0.001

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )
        # topic = f"/BASESTATION/{message_name}/{signal_name}"
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

        self.create_subscription(
            Float32, "/ARM/BASE/CURR_ANGLE", self.__on_base_angle_received, 10
        )
        self.create_subscription(
            Float32, "/ARM/SHOULDER/CURR_ANGLE", self.__on_shoulder_angle_received, 10
        )
        self.create_subscription(
            Float32, "/ARM/ELBOW/CURR_ANGLE", self.__on_elbow_angle_received, 10
        )
        self.create_subscription(
            Float32,
            "/ARM/WRIST_BEND/CURR_ANGLE",
            self.__on_wrist_bend_angle_received,
            10,
        )
        self.create_subscription(
            Float32,
            "/ARM/WRIST_TWIST/CURR_ANGLE",
            self.__on_wrist_twist_angle_received,
            10,
        )
        self.create_subscription(
            Float32, "/ARM/GRIPPER/CURR_ANGLE", self.__on_gripper_angle_received, 10
        )

        # create publishers for the arm motor angles
        self.__base_angle_publisher = self.create_publisher(
            Float32, "/ARM/BASE/TARGET_ANGLE", 10
        )
        self.__shoulder_angle_publisher = self.create_publisher(
            Float32, "/ARM/SHOULDER/TARGET_ANGLE", 10
        )
        self.__elbow_angle_publisher = self.create_publisher(
            Float32, "/ARM/ELBOW/TARGET_ANGLE", 10
        )
        self.__wrist_bend_angle_publisher = self.create_publisher(
            Float32, "/ARM/WRIST_BEND/TARGET_ANGLE", 10
        )
        self.__wrist_twist_angle_publisher = self.create_publisher(
            Float32, "/ARM/WRIST_TWIST/TARGET_ANGLE", 10
        )
        self.__gripper_angle_publisher = self.create_publisher(
            Float32, "/ARM/GRIPPER/TARGET_ANGLE", 10
        )

        # Update values periodically
        self.create_timer(
            timer_period_sec=UPDATE_RATE_SEC,
            callback=self.__calculate_angles,
        )

        self.create_subscription(
            msg_type=UInt16,
            topic=f"/BASESTATION/spacemouse/buttons",
            callback=self.__buttons_callback,
            qos_profile=10,
        )

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
        self.__homing_toggled = False
        self.__mode = ARM_MODES.RAW_CONTROL
        self.__mode_toggled = False

    def __on_estop_received(self, msg: Bool):
        if msg.data:
            self.__running = False
            self.get_logger().info("ESTOP received. Stopping arm controller.")
        else:
            self.get_logger().info("ESTOP cleared. Arm controller can be restarted.")

    def __on_x_received(self, msg: Float32):
        if msg.data != self.__x:
            self.__x = msg.data

    def __on_y_received(self, msg: Float32):
        if msg.data != self.__y:
            self.__y = msg.data

    def __on_z_received(self, msg: Float32):
        if msg.data != self.__z:
            self.__z = msg.data

    def __on_rx_received(self, msg: Float32):
        if msg.data != self.__rx:
            self.__rx = msg.data

    def __on_ry_received(self, msg: Float32):
        if msg.data != self.__ry:
            self.__ry = msg.data

    def __on_rz_received(self, msg: Float32):
        if msg.data != self.__rz:
            self.__rz = msg.data

    def __buttons_callback(self, msg: UInt16):
        self.get_logger().info(f"Button state received: {msg.data}")

        # Make it a toggle
        if msg.data & 0b10 and not self.__mode_toggled:
            self.__mode = ARM_MODES((self.__mode + 1) % 3)
            self.__mode_toggled = True
            if self.__mode == ARM_MODES.RAW_CONTROL:
                self.get_logger().info("Raw control mode enabled.")
            elif self.__mode == ARM_MODES.POINT_CONTROL:
                self.get_logger().info("Point control mode enabled.")
            else:
                self.get_logger().info("Gripper control mode enabled.")
        else:
            self.__mode_toggled = False

        if (msg.data & 0b01) and not self.__homing_toggled:
            self.__homing = bool(msg.data & 0b01)
            self.__homing_toggled = True
            if msg.data:
                self.get_logger().info("Homing initiated.")
            else:
                self.get_logger().info("Homing cleared.")

    def __on_base_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th0:
            self.get_logger().info(f"Base angle update received: {msg.data} radians")
            self.__curr_th0 = wrap_to_pi(msg.data)

    def __on_shoulder_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th1:
            self.get_logger().info(
                f"Shoulder angle update received: {msg.data} radians"
            )
            self.__curr_th1 = wrap_to_pi(msg.data)

    def __on_elbow_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th2:
            self.get_logger().info(f"Elbow angle update received: {msg.data} radians")
            self.__curr_th2 = wrap_to_pi(msg.data)

    def __on_wrist_twist_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th3:
            self.get_logger().info(
                f"Wrist twist angle update received: {msg.data} radians"
            )
            self.__curr_th3 = wrap_to_pi(msg.data)

    def __on_wrist_bend_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th4:
            self.get_logger().info(
                f"Wrist bend angle update received: {msg.data} radians"
            )
            self.__curr_th4 = wrap_to_pi(msg.data)

    def __on_gripper_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th5:
            self.get_logger().info(f"Gripper angle update received: {msg.data} radians")
            self.__curr_th5 = msg.data

    def __calculate_angles(self):
        # Verify that all angles have been initialized
        self._logger.info(
            f"Current angles: {self.__curr_th0}, {self.__curr_th1}, {self.__curr_th2}, {self.__curr_th3}, {self.__curr_th4}, {self.__curr_th5}"
        )
        if any(
            math.isnan(angle)
            for angle in [
                self.__curr_th0,
                self.__curr_th1,
                self.__curr_th2,
                self.__curr_th3,
                self.__curr_th4,
                self.__curr_th5,
            ]
        ):
            # self.get_logger().warning("Current angles not fully initialized. Cannot calculate angles.")
            return

        self._logger.info(
            f"Current angles: {self.__curr_th0}, {self.__curr_th1}, {self.__curr_th2}, {self.__curr_th3}, {self.__curr_th4}, {self.__curr_th5}"
        )
        self._logger.info(
            f"Current spacemouse state: x={self.__x}, y={self.__y}, z={self.__z}, rx={self.__rx}, ry={self.__ry}, rz={self.__rz}, homing={self.__homing}, mode={self.__mode}"
        )

        self.__target_th0 = self.__curr_th0
        self.__target_th1 = self.__curr_th1
        self.__target_th2 = self.__curr_th2
        self.__target_th3 = self.__curr_th3
        self.__target_th4 = self.__curr_th4
        self.__target_th5 = self.__curr_th5

        _, _, self.__point, _ = calc_joint_positions(
            self.__target_th0,
            self.__target_th1,
            self.__target_th2,
            self.__target_th3,
            False,
        )

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
        elif self.__mode == ARM_MODES.POINT_CONTROL:
            self.__target_th0 = wrap_to_pi(
                self.__target_th0 + self.__rx * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__target_th0,
                self.__target_th1,
                self.__target_th2,
                self.__target_th3,
                False,
            )

            self.__target_th1 = wrap_to_pi(
                self.__target_th1 + self.__ry * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__target_th0,
                self.__target_th1,
                self.__target_th2,
                self.__target_th3,
                False,
            )

            self.__target_th2 = wrap_to_pi(
                self.__target_th2 + self.__rz * self.__rotate_sens
            )
            _, _, _, self.__point = calc_joint_positions(
                self.__target_th0,
                self.__target_th1,
                self.__target_th2,
                self.__target_th3,
                False,
            )
        elif self.__mode == ARM_MODES.RAW_CONTROL:

            # Move point inside cube
            self.__point[0] += self.__x * self.__trans_sens
            self.__point[1] += self.__y * self.__trans_sens
            self.__point[2] += self.__z * self.__trans_sens

            # Compute arm joints
            try:
                self.__target_th0, self.__target_th1, self.__target_th2, _ = (
                    inverse_kinematics(
                        self.__point,
                        [
                            self.__target_th0,
                            self.__target_th1,
                            self.__target_th2,
                            self.__target_th3,
                        ],
                        self.__target_th4,
                    )
                )

            except Exception as e:
                self.get_logger().error(f"Inverse kinematics calculation failed: {e}")

        elif self.__mode == ARM_MODES.GRIPPER_CONTROL:
            # x rotates the gripper
            self.__target_th3 += self.__rx * self.__rotate_sens
            # z spins the gripper
            self.__target_th4 += self.__rz * self.__rotate_sens
            # y closes gripper
            self.__target_th5 += self.__ry * self.__rotate_sens

            # Compute arm joints
            try:
                self.__target_th0, self.__target_th1, self.__target_th2, _ = (
                    inverse_kinematics(
                        self.__point,
                        [
                            self.__target_th0,
                            self.__target_th1,
                            self.__target_th2,
                            self.__target_th3,
                        ],
                        self.__target_th4,
                    )
                )

            except Exception as e:
                self.get_logger().error(f"Inverse kinematics calculation failed: {e}")

        # # Reset state variable
        # for key in self.__state:
        #     self.__state[key] = 0

        # Update stuff
        (
            self.__target_th0,
            self.__target_th1,
            self.__target_th2,
            self.__target_th3,
            self.__target_th4,
        ) = joint_limits(
            self.__target_th0,
            self.__target_th1,
            self.__target_th2,
            self.__target_th3,
            self.__target_th4,
            not (self.__mode == ARM_MODES.POINT_CONTROL),
        )

        (
            self.__target_th0,
            self.__target_th1,
            self.__target_th2,
            self.__target_th3,
            self.__target_th4,
        ) = collision_protection(
            self.__target_th0,
            self.__target_th1,
            self.__target_th2,
            self.__target_th3,
            self.__target_th4,
        )
        _, _, l2, _ = calc_joint_positions(
            self.__target_th0, self.__target_th1, self.__target_th2, self.__target_th3
        )
        self.__point = l2

        # Must re-wrap some angles to prevent sign flips due to crossing the 180:-180 boundary
        self._logger.info(
            f"Calculated target angles: {wrap_to_minus_90(self.__target_th0)}, {wrap_to_minus_90(self.__target_th1)}, {wrap_to_pi(self.__target_th2)}, {wrap_to_pi(self.__target_th3)}, {wrap_to_pi(self.__target_th4)}, {wrap_to_pi(self.__target_th5)}"
        )
        self.__base_angle_publisher.publish(
            Float32(data=wrap_to_minus_90(self.__target_th0))
        )
        self.__shoulder_angle_publisher.publish(
            Float32(data=wrap_to_minus_90(self.__target_th1))
        )
        self.__elbow_angle_publisher.publish(
            Float32(data=wrap_to_pi(self.__target_th2))
        )
        self.__wrist_twist_angle_publisher.publish(
            Float32(data=wrap_to_pi(self.__target_th3))
        )
        self.__wrist_bend_angle_publisher.publish(
            Float32(data=wrap_to_pi(self.__target_th4))
        )
        self.__gripper_angle_publisher.publish(Float32(data=self.__target_th5))

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
