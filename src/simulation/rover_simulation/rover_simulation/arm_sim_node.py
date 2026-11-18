"""Simulated arm hardware node for Gazebo.

This node occupies the same role as ``arm_hardware_node`` on the real rover.
It subscribes to the normal arm target, enable, E-stop, and solenoid topics,
and publishes the normal ``/ARM/JOINT/CURRENT_*`` feedback topics.  The only
difference is the backend: commands go to the Gazebo ``arm_cont`` position
controller and feedback comes from the Gazebo joint-state broadcaster.

It is a hardware replacement, not a second command path.  The planner, master,
teleop, and typing nodes are identical between simulation and the rover.
"""

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray

from arm_constants import (
    ARM_ENABLED_TOPIC,
    ARM_JOINTS,
    ARM_MOTOR_IDX,
    CURRENT_ANGLE_TOPICS,
    ESTOP_TOPIC,
    GRIPPER_MAX_ANGLE,
    GRIPPER_MIN_ANGLE,
    HOMING_TARGET,
    MAX_ANG_VEL,
    MECHANICAL_LIMITS,
    SOLENOID_TARGET_TOPIC,
    TARGET_ANGLE_TOPICS,
)

# Match the Gazebo controller update rate.  The planner runs at DT = 0.05,
# but publishing interpolated commands at 30 Hz keeps position control smoother.
# The planner can land exactly on a limit after IK and floating-point
# arithmetic, so accept a hair outside before clamping back into range.
LIMIT_TOLERANCE_RAD = 1e-4
TICK_PERIOD_SEC = 1.0 / 30.0
# Allow the simulated motors a little headroom over the planner's slew rate.
# The planner already rate-limits its targets, but this keeps the simulated
# joints from teleporting after a start-up pose, a rejected target, or a mode
# change without making the arm lag behind a normal move.
MAX_COMMAND_STEP_RAD = 2.0 * MAX_ANG_VEL * TICK_PERIOD_SEC
SETTLE_TOLERANCE_RAD = 0.05
MAX_SETTLE_TICKS = 300

# Joint order is the same as rover_simulation/config/my_controllers.yaml.
GAZEBO_JOINTS = [
    "arm_base_joint",
    "arm_shoulder_joint",
    "arm_elbow_joint",
    "arm_wrist_bend_joint",
    "arm_wrist_twist_joint",
    "arm_gripper_joint",
]

# Simulation-only randomized start: base/shoulder/elbow at home +/- 20 deg.
# Wrist bend, wrist twist, and gripper always start at zero.
START_JITTER_RAD = math.radians(20.0)
_HOME = list(HOMING_TARGET) + [0.0]
# Use the mechanical limits rather than the encoder calibration.  The tick
# constants describe the real motors and are intentionally not part of the
# simulated hardware contract.
_LIMITS = [
    MECHANICAL_LIMITS[0],
    MECHANICAL_LIMITS[1],
    MECHANICAL_LIMITS[2],
    MECHANICAL_LIMITS[3],
    MECHANICAL_LIMITS[4],
    (GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE),
]


def joint_distance(index, current: float, target: float) -> float:
    """Return the shortest angular distance for one joint."""
    diff = float(target) - float(current)
    if index == ARM_MOTOR_IDX.BASE:
        diff = math.remainder(diff, 2.0 * math.pi)
    return abs(diff)


def random_home_pose():
    """Return a randomized, mechanically valid start pose.

    Only the three main joints jitter around home.  The wrists and gripper
    always start at zero.
    """
    angles = []
    for home, (low, high) in zip(_HOME[:3], _LIMITS[:3]):
        angle = home + random.uniform(-START_JITTER_RAD, START_JITTER_RAD)
        angles.append(max(low, min(high, angle)))
    angles.extend([0.0, 0.0, 0.0])
    return angles


class ArmSimNode(Node):
    """Run the arm's normal ROS interface on top of Gazebo joints."""

    def __init__(self):
        """Create the hardware topics and the Gazebo command/feedback path."""
        super().__init__("arm_sim_node")

        self.declare_parameter(
            "command_topic", "/GAZEBO/ARM/arm_cont/commands")
        self.declare_parameter(
            "joint_state_topic", "/GAZEBO/joint_states")
        self.declare_parameter("randomize_start", False)

        command_topic = self.get_parameter(
            "command_topic").get_parameter_value().string_value
        joint_state_topic = self.get_parameter(
            "joint_state_topic").get_parameter_value().string_value
        self.__randomize_start = self.get_parameter(
            "randomize_start").get_parameter_value().bool_value

        self.__arm_enabled = False
        self.__estop = False
        self.__enable_ignored_logged = False
        self.__started = False
        self.__settling = False
        self.__settle_ticks = 0

        self.__targets = [float("nan")] * len(GAZEBO_JOINTS)
        self.__commands = [float("nan")] * len(GAZEBO_JOINTS)
        self.__current = [float("nan")] * len(GAZEBO_JOINTS)

        self.__command_publisher = self.create_publisher(
            Float64MultiArray, command_topic, 10)
        self.__current_publishers = {
            index: self.create_publisher(
                Float32, CURRENT_ANGLE_TOPICS[index], 10)
            for index in ARM_MOTOR_IDX
        }

        self.create_subscription(
            JointState, joint_state_topic, self.__on_joint_state, 10)
        self.create_subscription(
            Bool, ARM_ENABLED_TOPIC, self.__on_arm_enable_received, 10)
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop_received, 10)
        self.create_subscription(
            Bool,
            SOLENOID_TARGET_TOPIC,
            self.__on_solenoid_received,
            10,
        )
        for index in ARM_MOTOR_IDX:
            self.create_subscription(
                Float32,
                TARGET_ANGLE_TOPICS[index],
                self.__target_callback(index),
                10,
            )

        self.create_timer(TICK_PERIOD_SEC, self.__tick)

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def __target_callback(self, index: ARM_MOTOR_IDX):
        """Build a callback that validates and latches one joint target."""
        def callback(msg: Float32):
            if self.__estop or not self.__arm_enabled or self.__settling:
                return
            self.__set_target(index, float(msg.data))
        return callback

    def __set_target(self, index: ARM_MOTOR_IDX, angle: float):
        """Validate one target against the mechanical joint limits."""
        if not math.isfinite(angle):
            self.get_logger().error(
                f"Target for {ARM_JOINTS[index]} is not finite")
            return
        # The planner wraps the continuous base with wrap_to_minus_90, so a
        # valid -100 deg target can arrive as +260 deg.  Normalise it back to
        # [-pi, pi] before checking the mechanical limits; otherwise every
        # base target below -90 deg is rejected and the joint stutters.
        angle = math.remainder(angle, 2.0 * math.pi)
        low, high = _LIMITS[index]
        if (angle < low - LIMIT_TOLERANCE_RAD
                or angle > high + LIMIT_TOLERANCE_RAD):
            self.get_logger().error(
                f"Target {angle:.4f} rad for {ARM_JOINTS[index]} is out of "
                f"limits [{low:.4f}, {high:.4f}] rad")
            return
        self.__targets[index] = max(low, min(high, angle))

    def __on_arm_enable_received(self, msg: Bool):
        """Enable or disable simulated motion, honouring the E-stop latch."""
        if self.__estop and msg.data:
            if not self.__enable_ignored_logged:
                self.__enable_ignored_logged = True
                self.get_logger().warning(
                    "Ignoring arm enable while E-STOP is latched")
            return

        if not msg.data:
            if not self.__arm_enabled:
                return
            self.__arm_enabled = False
            self.__settling = False
            self.__hold_current()
            self.get_logger().info("Arm disabled")
            return

        # sim_config_node republishes this level continuously.  Only reset the
        # targets on the false -> true edge; resetting on every repeat would
        # cancel an in-progress move once per second.
        if not self.__arm_enabled:
            self.__arm_enabled = True
            self.__hold_current()
            self.get_logger().info("Arm enabled")

    def __on_estop_received(self, msg: Bool):
        """Latch or clear the E-stop and disable simulated motion."""
        if not msg.data:
            if not self.__estop:
                return
            self.__estop = False
            self.__enable_ignored_logged = False
            self.get_logger().info("E-STOP cleared: arm can be re-enabled")
            return

        if self.__estop:
            return
        self.__estop = True
        self.__arm_enabled = False
        self.__settling = False
        self.__hold_current()
        self.get_logger().info("E-STOP latched: holding simulated arm pose")

    def __on_solenoid_received(self, msg: Bool):
        """Accept the normal solenoid pulse; Gazebo has no striker model."""
        if msg.data and self.__arm_enabled:
            self.get_logger().debug("Simulated solenoid strike")

    def __on_joint_state(self, msg: JointState):
        """Latch Gazebo joint positions for the current-angle feedback."""
        for name, position in zip(msg.name, msg.position):
            if name in GAZEBO_JOINTS:
                self.__current[GAZEBO_JOINTS.index(name)] = float(position)

    # ------------------------------------------------------------------
    # Gazebo outputs
    # ------------------------------------------------------------------

    def __hold_current(self):
        """Use the measured pose as the new target."""
        if all(math.isfinite(value) for value in self.__current):
            self.__targets = list(self.__current)

    def __publish_targets(self):
        """Slew the command toward the target and send it to Gazebo."""
        if (any(math.isnan(value) for value in self.__targets)
                or any(math.isnan(value) for value in self.__commands)):
            return
        commands = list(self.__commands)
        for index, target in enumerate(self.__targets):
            diff = target - commands[index]
            if index == ARM_MOTOR_IDX.BASE:
                # The base is continuous.  Take the shortest angular path.
                diff = math.remainder(diff, 2 * math.pi)
            diff = max(-MAX_COMMAND_STEP_RAD,
                       min(MAX_COMMAND_STEP_RAD, diff))
            commands[index] += diff
        self.__commands = commands
        self.__command_publisher.publish(
            Float64MultiArray(data=commands))

    def __publish_current(self):
        """Publish the Gazebo joint positions as arm feedback."""
        for index, value in enumerate(self.__current):
            if math.isfinite(value):
                self.__current_publishers[ARM_MOTOR_IDX(index)].publish(
                    Float32(data=value))

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    def __tick(self):
        """Publish feedback and refresh the Gazebo joint targets."""
        if any(math.isnan(value) for value in self.__current):
            return

        self.__publish_current()

        if not self.__started:
            if self.__estop or not self.__arm_enabled:
                return
            if self.__randomize_start:
                self.__targets = random_home_pose()
                self.__commands = list(self.__current)
                self.__settling = True
                self.get_logger().info(
                    "Simulated arm starting from randomized angles "
                    f"{[round(math.degrees(a), 1) for a in self.__targets]}")
            else:
                self.__targets = list(self.__current)
                self.__commands = list(self.__current)
            self.__started = True

        if self.__estop or not self.__arm_enabled:
            self.__hold_current()

        self.__publish_targets()

        if self.__settling:
            self.__settle_ticks += 1
            reached = all(
                joint_distance(index, current, target)
                <= SETTLE_TOLERANCE_RAD
                for index, (current, target)
                in enumerate(zip(self.__current, self.__targets)))
            if not reached and self.__settle_ticks < MAX_SETTLE_TICKS:
                return
            self.__settling = False
            if not reached:
                self.get_logger().warning(
                    "Simulated arm did not reach the randomized start pose; "
                    "resuming normal targets")
            else:
                self.get_logger().info(
                    "Simulated arm reached its randomized start pose")

    def run(self):
        """Spin the node until shutdown."""
        self.get_logger().info("Simulated arm hardware started")
        rclpy.spin(self)
        self.get_logger().info("Simulated arm hardware stopped")


def main():
    """Create and run the simulated arm hardware node."""
    rclpy.init()
    node = None
    try:
        node = ArmSimNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
