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
    ARM_MOTOR_PARAMS,
    CURRENT_ANGLE_TOPICS,
    ESTOP_TOPIC,
    GRIPPER_MAX_ANGLE,
    GRIPPER_MIN_ANGLE,
    HOMING_TARGET,
    MECHANICAL_LIMITS,
    SOLENOID_TARGET_TOPIC,
    TARGET_ANGLE_TOPICS,
)

# Match the Gazebo controller update rate.  The planner runs at DT = 0.05,
# but publishing interpolated commands at 30 Hz keeps position control
# smoother.  The simulated hardware works in encoder ticks, using the same
# calibration, limits, and motor speed limits as arm_hardware_node.
LIMIT_TOLERANCE_RAD = 1e-4
TICK_PERIOD_SEC = 1.0 / 30.0
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


def angle_to_ticks(index: int, angle: float) -> float:
    """Convert a joint angle [rad] to encoder ticks."""
    params = ARM_MOTOR_PARAMS[index]
    return float(angle) * params.ticks_per_rad + params.tick_offset


def ticks_to_angle(index: int, ticks: float) -> float:
    """Convert encoder ticks to a joint angle [rad]."""
    params = ARM_MOTOR_PARAMS[index]
    return (float(ticks) - params.tick_offset) / params.ticks_per_rad


def _tick_limits(index: int) -> tuple[float, float]:
    """Return the hardware tick limits for one joint, ordered."""
    params = ARM_MOTOR_PARAMS[index]
    return (
        min(params.tick_lower_limit, params.tick_upper_limit),
        max(params.tick_lower_limit, params.tick_upper_limit),
    )


TICK_LIMITS = [_tick_limits(index) for index in range(len(GAZEBO_JOINTS))]
LIMIT_TOLERANCE_TICKS = [
    LIMIT_TOLERANCE_RAD * abs(ARM_MOTOR_PARAMS[index].ticks_per_rad)
    for index in range(len(GAZEBO_JOINTS))
]
TICKS_PER_REV = [
    abs(ARM_MOTOR_PARAMS[index].ticks_per_rad) * 2.0 * math.pi
    for index in range(len(GAZEBO_JOINTS))
]
MAX_COMMAND_STEP_TICKS = [
    ARM_MOTOR_PARAMS[index].max_ticks_per_s * TICK_PERIOD_SEC
    for index in range(len(GAZEBO_JOINTS))
]
SETTLE_TOLERANCE_TICKS = [
    SETTLE_TOLERANCE_RAD * abs(ARM_MOTOR_PARAMS[index].ticks_per_rad)
    for index in range(len(GAZEBO_JOINTS))
]

# Simulation-only randomized start: base/shoulder/elbow at home +/- 20 deg.
# Wrist bend, wrist twist, and gripper always start at zero.
START_JITTER_RAD = math.radians(20.0)
_HOME = list(HOMING_TARGET) + [0.0]
# Random start poses stay inside the mechanical limits; the tick limits are
# used for command validation because they are what the hardware enforces.
_LIMITS = [
    MECHANICAL_LIMITS[0],
    MECHANICAL_LIMITS[1],
    MECHANICAL_LIMITS[2],
    MECHANICAL_LIMITS[3],
    MECHANICAL_LIMITS[4],
    (GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE),
]


def tick_distance(index: int, current: float, target: float) -> float:
    """Return the shortest tick distance for one joint."""
    diff = float(target) - float(current)
    if index == ARM_MOTOR_IDX.BASE:
        diff = math.remainder(diff, TICKS_PER_REV[index])
    return abs(diff)


def random_initial_angles(jitter=START_JITTER_RAD):
    """Return randomized, mechanically valid 5-DOF start angles.

    Every arm joint jitters around its home pose and is clamped to the
    mechanical limits.
    """
    angles = []
    for home, (low, high) in zip(_HOME[:5], _LIMITS[:5]):
        angle = home + random.uniform(-jitter, jitter)
        angles.append(max(low, min(high, angle)))
    return angles


def random_home_pose():
    """Return a randomized, mechanically valid start pose.

    Only the three main joints jitter around home.  The wrists and gripper
    always start at zero.
    """
    angles = random_initial_angles()
    angles[3] = 0.0
    angles[4] = 0.0
    return angles + [0.0]


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

        self.__targets_ticks = [float("nan")] * len(GAZEBO_JOINTS)
        self.__commands_ticks = [float("nan")] * len(GAZEBO_JOINTS)
        self.__current_ticks = [float("nan")] * len(GAZEBO_JOINTS)

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
        """Validate one target against the hardware tick limits."""
        if not math.isfinite(angle):
            self.get_logger().error(
                f"Target for {ARM_JOINTS[index]} is not finite")
            return
        # Normalise the continuous base before checking the limits.  The
        # planner already wraps every joint around zero, so this is
        # defensive: an externally published 0..2pi value still lands on the
        # matching angle instead of being rejected.
        angle = math.remainder(angle, 2.0 * math.pi)
        ticks = angle_to_ticks(index, angle)
        low, high = TICK_LIMITS[index]
        tolerance = LIMIT_TOLERANCE_TICKS[index]
        if ticks < low - tolerance or ticks > high + tolerance:
            low_rad = ticks_to_angle(index, low)
            high_rad = ticks_to_angle(index, high)
            self.get_logger().error(
                f"Target {angle:.4f} rad ({ticks:.0f} ticks) for "
                f"{ARM_JOINTS[index]} is out of limits "
                f"[{low_rad:.4f}, {high_rad:.4f}] rad "
                f"([{low:.0f}, {high:.0f}] ticks)")
            return
        self.__targets_ticks[index] = max(low, min(high, ticks))

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
                index = GAZEBO_JOINTS.index(name)
                self.__current_ticks[index] = angle_to_ticks(
                    index, float(position))

    # ------------------------------------------------------------------
    # Gazebo outputs
    # ------------------------------------------------------------------

    def __hold_current(self):
        """Use the measured pose as the new target."""
        if all(math.isfinite(value) for value in self.__current_ticks):
            self.__targets_ticks = list(self.__current_ticks)

    def __publish_targets(self):
        """Slew the command ticks toward the target and send radians."""
        if (any(math.isnan(value) for value in self.__targets_ticks)
                or any(math.isnan(value) for value in self.__commands_ticks)):
            return
        commands = list(self.__commands_ticks)
        for index, target in enumerate(self.__targets_ticks):
            diff = target - commands[index]
            if index == ARM_MOTOR_IDX.BASE:
                # The base is continuous.  Take the shortest tick path.
                diff = math.remainder(diff, TICKS_PER_REV[index])
            step = MAX_COMMAND_STEP_TICKS[index]
            diff = max(-step, min(step, diff))
            low, high = TICK_LIMITS[index]
            commands[index] = max(low, min(high, commands[index] + diff))
        self.__commands_ticks = commands
        self.__command_publisher.publish(Float64MultiArray(
            data=[
                ticks_to_angle(index, ticks)
                for index, ticks in enumerate(commands)
            ]))

    def __publish_current(self):
        """Publish the Gazebo joint positions as arm feedback."""
        for index, ticks in enumerate(self.__current_ticks):
            if math.isfinite(ticks):
                self.__current_publishers[ARM_MOTOR_IDX(index)].publish(
                    Float32(data=ticks_to_angle(index, ticks)))

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    def __tick(self):
        """Publish feedback and refresh the Gazebo joint targets."""
        if any(math.isnan(value) for value in self.__current_ticks):
            return

        self.__publish_current()

        if not self.__started:
            if self.__estop or not self.__arm_enabled:
                return
            if self.__randomize_start:
                target_angles = random_home_pose()
                self.__targets_ticks = [
                    angle_to_ticks(index, angle)
                    for index, angle in enumerate(target_angles)
                ]
                self.__commands_ticks = list(self.__current_ticks)
                self.__settling = True
                self.get_logger().info(
                    "Simulated arm starting from randomized angles "
                    f"{[round(math.degrees(a), 1) for a in target_angles]}")
            else:
                self.__targets_ticks = list(self.__current_ticks)
                self.__commands_ticks = list(self.__current_ticks)
            self.__started = True

        if self.__estop or not self.__arm_enabled:
            self.__hold_current()

        self.__publish_targets()

        if self.__settling:
            self.__settle_ticks += 1
            reached = all(
                tick_distance(index, current, target)
                <= SETTLE_TOLERANCE_TICKS[index]
                for index, (current, target)
                in enumerate(zip(self.__current_ticks, self.__targets_ticks)))
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
