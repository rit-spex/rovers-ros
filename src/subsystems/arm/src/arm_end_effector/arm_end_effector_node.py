"""Arm end effector node: gripper position and solenoid strikes.

Owns every gripper and solenoid runtime concern: the gripper target, the
up/down arrow jog, and the momentary solenoid pulse.  It has no kinematic
model: the 5-DOF arm stays in ``arm_route_planner_node``, and
``arm_master_node`` validates this node's command preconditions and forwards
only the commands that belong here.  It does not subscribe to the control
source: the master muxes ownership and emits an explicit ``stop_jog`` on a
handover.

The basestation keyboard sends ``gripper_up`` / ``gripper_down`` while the
arrow is held.  Each command extends a short jog deadline, so a dead input
stream stops the gripper instead of leaving it creeping.  The planner is not
involved and the SpaceMouse axes are pure motion input.

It publishes the gripper target whenever a measured angle is available;
``arm_hardware_node`` (and the Gazebo bridge) drop targets while the arm is
disabled.

Inputs:
    /ESTOP                      std_msgs/Bool
    /ARM/END_EFFECTOR/COMMAND   custom_interfaces/ArmCommand
    /ARM/JOINT/CURRENT_GRIPPER  std_msgs/Float32

Outputs:
    /ARM/JOINT/TARGET_GRIPPER   std_msgs/Float32
    /ARM/JOINT/TARGET_SOLENOID  std_msgs/Bool
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from custom_interfaces.msg import ArmCommand
from std_msgs.msg import Bool, Float32

from arm_constants import (
    DT,
    END_EFFECTOR_COMMAND_TOPIC,
    END_EFFECTOR_STOP_COMMAND,
    ESTOP_TOPIC,
    GRIPPER_JOG_RATE,
    GRIPPER_JOG_TIMEOUT_SEC,
    GRIPPER_MAX_ANGLE,
    GRIPPER_MIN_ANGLE,
    JOINT_CURRENT_TOPICS,
    JOINT_TARGET_TOPICS,
    SOLENOID_TARGET_TOPIC,
)

# 0 rad is open and pi/2 is closed, so up opens and down closes.
GRIPPER_JOG_DIRECTIONS = {
    "gripper_up": -1.0,
    "gripper_down": 1.0,
}


class ArmEndEffectorNode(Node):
    """Drive the gripper and solenoid."""

    def __init__(self):
        super().__init__("arm_end_effector_node")

        self.__estop = False

        # Gripper state.
        self.__measured_gripper = float("nan")
        self.__gripper_target = float("nan")
        self.__jog_direction = 0.0
        self.__jog_until = 0.0

        # Publishing bookkeeping.
        self.__last_gripper_target = None
        self.__publish_counter = 0

        self.__set_publishers()
        self.__set_subscribers()
        self.create_timer(DT, self.__update)

    # ------------------------------------------------------------------
    # ROS interfaces
    # ------------------------------------------------------------------

    def __set_publishers(self):
        """Create the gripper and solenoid publishers."""
        self.__gripper_angle_publisher = self.create_publisher(
            Float32, JOINT_TARGET_TOPICS["gripper"], 10)
        self.__solenoid_publisher = self.create_publisher(
            Bool, SOLENOID_TARGET_TOPIC, 10)

    def __set_subscribers(self):
        """Create the E-stop, command, and gripper subscriptions."""
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop_received, 10)
        self.create_subscription(
            ArmCommand,
            END_EFFECTOR_COMMAND_TOPIC,
            self.__on_command_received,
            10,
        )
        self.create_subscription(
            Float32,
            JOINT_CURRENT_TOPICS["gripper"],
            self.__on_gripper_angle_received,
            10,
        )

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def __on_estop_received(self, msg: Bool):
        estop = bool(msg.data)
        if estop == self.__estop:
            return
        self.__estop = estop
        if self.__estop:
            self.__jog_direction = 0.0
            self.get_logger().info(
                "E-STOP latched: end effector paused")
        else:
            self.get_logger().info(
                "E-STOP cleared: end effector resumed")

    def __on_gripper_angle_received(self, msg: Float32):
        self.__measured_gripper = float(msg.data)
        # Track the hardware unless the node is actively jogging it.
        if self.__jog_direction == 0.0 or math.isnan(self.__gripper_target):
            self.__gripper_target = self.__clamp_gripper(
                self.__measured_gripper)

    def __on_command_received(self, msg: ArmCommand):
        command = msg.command.strip()
        if not command or self.__estop:
            return
        if command == END_EFFECTOR_STOP_COMMAND:
            self.__stop_gripper_jog()
        elif command in GRIPPER_JOG_DIRECTIONS:
            self.__start_gripper_jog(command)
        elif command in ("fire_solenoid", "toggle_solenoid"):
            self.__fire_solenoid()

    # ------------------------------------------------------------------
    # Actuators
    # ------------------------------------------------------------------

    def __start_gripper_jog(self, command: str):
        """Latch a jog direction and extend its deadline."""
        self.__jog_direction = GRIPPER_JOG_DIRECTIONS[command]
        self.__jog_until = time.monotonic() + GRIPPER_JOG_TIMEOUT_SEC

    def __stop_gripper_jog(self):
        """Release a held jog and hold the measured angle.

        The master sends this on a control handover, so the actuator does not
        need to know anything about control ownership.
        """
        if self.__jog_direction == 0.0:
            return
        self.__jog_direction = 0.0
        self.__gripper_target = self.__measured_gripper
        self.get_logger().info("Gripper jog released")

    def __fire_solenoid(self):
        """Fire one solenoid strike.

        The Teensy drives the solenoid and retracts it on its own, so this
        is a single command pulse rather than a state the rover tracks.
        """
        self.__solenoid_publisher.publish(Bool(data=True))
        self.get_logger().info("Solenoid strike")

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    @staticmethod
    def __clamp_gripper(angle):
        return max(GRIPPER_MIN_ANGLE, min(GRIPPER_MAX_ANGLE, angle))

    def __update_gripper_jog(self):
        if self.__jog_direction == 0.0:
            return
        if time.monotonic() > self.__jog_until:
            # The arrow stream stopped; hold position instead of creeping.
            self.__jog_direction = 0.0
            return
        if math.isnan(self.__measured_gripper):
            return
        if math.isnan(self.__gripper_target):
            self.__gripper_target = self.__measured_gripper
        self.__gripper_target = self.__clamp_gripper(
            self.__gripper_target
            + self.__jog_direction * GRIPPER_JOG_RATE * DT)

    def __publish_gripper_target(self, force=False):
        gripper = (None if math.isnan(self.__gripper_target)
                   else float(self.__gripper_target))
        if gripper == self.__last_gripper_target and not force:
            return
        self.__last_gripper_target = gripper
        if gripper is not None:
            self.__gripper_angle_publisher.publish(Float32(data=gripper))

    def __update(self):
        if self.__estop:
            return

        self.__publish_counter += 1
        self.__update_gripper_jog()
        self.__publish_gripper_target(
            force=self.__publish_counter % 20 == 0)

    def run(self):
        self.get_logger().info("Arm end effector started")
        rclpy.spin(self)
        self.get_logger().info("Arm end effector stopped")


def main():
    rclpy.init()
    end_effector = None
    try:
        end_effector = ArmEndEffectorNode()
        end_effector.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if end_effector is not None:
            end_effector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
