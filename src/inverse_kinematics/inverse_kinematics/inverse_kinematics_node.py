import math
from linear_algebra import (
    calc_joint_positions,
    collision_protection,
    homingStep,
    inverse_kinematics,
    joint_limits,
)

from math_helpers import wrap_to_pi, wrap_to_minus_90
from fast_IK import forward_kin, fast_IK_solve
import rclpy
import time

# Typing
from rclpy.node import Node
from typing import Any
from numpy import dtype, ndarray

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState

from rclpy.subscription import Subscription
from std_msgs.msg import Bool, Float32, Int16, UInt16
from enum import IntEnum

from constants.CommandCodes import CONSTANTS
from custom_interfaces.msg import ArmWrist


"""

Run this to viz arm
sudo apt install ros-humble-urdf-tutorial
ros2 launch urdf_tutorial display.launch.py model:=$(pwd)/arm.urdf jsp_gui:=false

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

UPDATE_RATE_SEC = 1.0 / 18.0  # seconds


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
    __solenoid: bool
    __solenoid_toggled: bool
    __mode: ARM_MODES  # Buttonz
    __mode_toggled: bool

    # General parameters
    __point: list[float]  # ndarray[tuple[int], dtype[Any]]
    __trans_sens: float
    __rotate_sens: float

    # Node parameters
    __base_angle_publisher: Publisher
    __shoulder_angle_publisher: Publisher
    __elbow_angle_publisher: Publisher
    __wrist_angle_publisher: Publisher
    __gripper_angle_publisher: Publisher
    __solenoid_publisher: Publisher

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

        self.__target_th3 = float("nan")
        self.__target_th4 = float("nan")
        self.__target_th5 = float("nan")

        self.A0 = 6.5 / 39.0  # Elevation of shoulder joint from base [in]
        self.A1 = 18.5 / 39.0  # Upper arm length [in]
        self.A2 = 19.75 / 39.0  # Fore-arm length [in]
        self.A3 = 11.5 / 39.0  # Gripper length [in]

        self.__initialized = False

        self.__x = 0
        self.__y = 0
        self.__z = 0
        self.__rx = 0
        self.__ry = 0
        self.__rz = 0

        self.__trans_sens = 0.1 * (1.0 / 350.0)
        self.__rotate_sens = 1.0 * (1.0 / 350.0)

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
        self.__wrist_angle_publisher = self.create_publisher(
            ArmWrist, "/ARM/WRIST/TARGET_ANGLE", 10
        )
        self.__gripper_angle_publisher = self.create_publisher(
            Float32, "/ARM/GRIPPER/TARGET_ANGLE", 10
        )
        self.__solenoid_publisher = self.create_publisher(
            Bool, "/ARM/SOLENOID/ENABLED", 10
        )
        self.__joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
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
        self.__solenoid = False
        self.__solenoid_toggled = False
        self.__homing = False
        self.__homing_toggled = False
        self.__mode = ARM_MODES.POINT_CONTROL
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
        elif not msg.data & 0b10:
            self.__mode_toggled = False

        if (msg.data & 0b01) and not self.__solenoid_toggled:
            self.__solenoid = not self.__solenoid
            new_msg = Bool()
            new_msg.data = self.__solenoid
            self.__solenoid_publisher.publish(msg=new_msg)
            if self.__solenoid:
                self.get_logger().info("Solenoid set.")
            else:
                self.get_logger().info("Solenoid cleared.")
        elif not (msg.data & 0b01):
            self.__solenoid_toggled = False

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
        if msg.data != self.__curr_th4:
            self.get_logger().info(
                f"Wrist twist angle update received: {msg.data} radians"
            )
            self.__curr_th4 = wrap_to_pi(msg.data)

    def __on_wrist_bend_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th3:
            self.get_logger().info(
                f"Wrist bend angle update received: {msg.data} radians"
            )
            self.__curr_th3 = wrap_to_pi(msg.data)

    def __on_gripper_angle_received(self, msg: Float32):
        if msg.data != self.__curr_th5:
            self.get_logger().info(f"Gripper angle update received: {msg.data} radians")
            self.__curr_th5 = msg.data

    def __calculate_angles(self):
        start_time = time.time()

        # Verify that all angles have been initialized
        self._logger.info(
            f"Current angles: {self.__curr_th0 *57.3}, {self.__curr_th1*57.3}, {self.__curr_th2*57.3}, {self.__curr_th3*57.3}, {self.__curr_th4*57.3}, {self.__curr_th5*57.3}"
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

        if not self.__initialized:
            self.__point = forward_kin(
                [self.__curr_th0, self.__curr_th1, self.__curr_th2, self.__curr_th3],
                self.A0,
                self.A1,
                self.A2,
                self.A3,
            )
            self.__initialized = True

            self.__target_th3 = self.__curr_th3
            self.__target_th4 = self.__curr_th4
            self.__target_th5 = self.__curr_th5

            self._logger.info("")

        self._logger.info(
            f"Current spacemouse state: x={self.__x}, y={self.__y}, z={self.__z}, rx={self.__rx}, ry={self.__ry}, rz={self.__rz}, homing={self.__homing}, mode={self.__mode}"
        )

        self.__target_th0 = self.__curr_th0
        self.__target_th1 = self.__curr_th1
        self.__target_th2 = self.__curr_th2
        # self.__target_th3 = self.__curr_th3
        # self.__target_th4 = self.__curr_th4
        # self.__target_th5 = self.__curr_th5

        end_time = time.time()
        self.get_logger().info(f"Time before calcs: {end_time - start_time}:.3f")

        if self.__mode == ARM_MODES.POINT_CONTROL:
            # P_new = P_old + (Velocity_Command * dt)
            self.__point[0] += (self.__x * self.__trans_sens) * UPDATE_RATE_SEC
            self.__point[1] += (self.__y * self.__trans_sens) * UPDATE_RATE_SEC
            self.__point[2] += (self.__z * self.__trans_sens) * UPDATE_RATE_SEC

            self.__target_th3 += (self.__rx * self.__rotate_sens) * UPDATE_RATE_SEC
            self.__target_th3 = min(max(self.__target_th3, -0.5), 0.5)

            self.__target_th4 += (self.__rz * self.__rotate_sens) * UPDATE_RATE_SEC
            self.__target_th4 = min(max(self.__target_th4, -1.5), 1.5)

            self.__target_th5 += (self.__ry * self.__rotate_sens) * UPDATE_RATE_SEC
            self.__target_th5 = min(max(self.__target_th5, 0), 1.57)

            # IK Logic
            current_ang = [self.__curr_th0, self.__curr_th1, self.__curr_th2]
            self.__target_th0, self.__target_th1, self.__target_th2 = fast_IK_solve(
                current_ang,
                self._logger,
                self.__point,
                self.__target_th3,
                self.A0,
                self.A1,
                self.A2,
                self.A3,
            )

        elif self.__mode == ARM_MODES.RAW_CONTROL:
            self.__mode = ARM_MODES.POINT_CONTROL
            return

        elif self.__mode == ARM_MODES.GRIPPER_CONTROL:
            self.__mode = ARM_MODES.POINT_CONTROL
            return

        end_time = time.time()
        self.get_logger().info(f"Time after calcs: {end_time - start_time}:.3f")

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

        """(
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
        )"""

        # Publish Calculated Angles
        self._logger.info(
            f"Calculated target angles: {(self.__target_th0)}, {(self.__target_th1)}, {(self.__target_th2)}, {(self.__target_th3)}, {(self.__target_th4)}, {(self.__target_th5)}"
        )

        self.__base_angle_publisher.publish(Float32(data=self.__target_th0))
        self.__shoulder_angle_publisher.publish(Float32(data=self.__target_th1))
        self.__elbow_angle_publisher.publish(Float32(data=self.__target_th2))
        self.__wrist_angle_publisher.publish(
            ArmWrist(
                wrist_bend=self.__target_th3,
                wrist_twist=self.__target_th4,
            )
        )
        self.__gripper_angle_publisher.publish(
            Float32(data=float(self.__target_th5 * 1.00001))
        )

        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        # These names MUST match the <joint name="..."> in your custom_arm.urdf
        js_msg.name = ["joint0_base", "joint1_shoulder", "joint2_elbow", "joint3_wrist"]
        # Map your calculated IK angles to those joints
        js_msg.position = [
            float(self.__target_th0),
            float(self.__target_th1),
            float(self.__target_th2),
            float(self.__target_th3),
        ]
        self.__joint_state_publisher.publish(js_msg)

        # Timing
        end_time = time.time()
        self.get_logger().info(f"Time To update: {end_time - start_time}:.3f")

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
