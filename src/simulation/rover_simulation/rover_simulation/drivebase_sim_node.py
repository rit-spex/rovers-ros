"""Simulated drivebase hardware node for Gazebo.

This node occupies the same role as the chassis Teensy on the real rover.  It
subscribes to the ``Drive_Power`` CAN frame that ``chassis_node`` publishes,
turns the per-side motor power into a diff-drive twist, and drives the Gazebo
``diff_cont`` controller.

It also keeps the simulation-only command sources that existed around the old
bridge chain: ``/cmd_vel`` navigation and the Gazebo joystick topic.  Priority
matches the previous setup: E-stop > CAN drive power > joystick > navigation.
The node applies the drive-motor deadzone and the CAN command timeout itself,
so there is no separate twist mux or relay stage.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from constants.CAN_constants import CAN_MESSAGE_IDS, TEENSY_CAN_MESSAGES
from constants.can_encoding import TeensyCommunication
from custom_interfaces.msg import Can

INPUT_TOPIC = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.DRIVE_POWER].topic_name
NAV_TOPIC = "/cmd_vel"
JOYSTICK_TOPIC = "/GAZEBO/CHASSIS/diff_cont/cmd_vel_joy"
ESTOP_TOPIC = "/ESTOP"
# diff_drive_controller subscribes to cmd_vel_unstamped with BEST_EFFORT.
OUTPUT_TOPIC = "/GAZEBO/CHASSIS/diff_cont/cmd_vel_unstamped"

PUBLISH_PERIOD_SEC = 0.1
# Keep publishing a released CAN command briefly so a stop is not lost while
# a lower-priority source takes over.
RELEASE_HOLD_SEC = 0.3
# Drive_Power frames are streamed.  Silence this long means the command path
# is gone, so stop treating the latched command as current.
COMMAND_TIMEOUT_SEC = 1.0
# Navigation and joystick use the same 0.5 s freshness timeout.
MUX_TIMEOUT_SEC = 0.5

# Keep in sync with config/my_controllers.yaml.
WHEEL_SEPARATION_DEFAULT = 0.58
WHEEL_RADIUS_DEFAULT = 0.081674
# Matches the wheel velocity command limit in ros2_control.xacro.
MAX_WHEEL_ANGULAR_VELOCITY_DEFAULT = 10.0
# Small drive-motor deadband at the wheel [m/s].
MOTOR_DEADZONE_DEFAULT = 0.1

BEST_EFFORT_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def drive_power_to_twist(
    left: float,
    right: float,
    wheel_separation: float,
    wheel_radius: float,
    max_wheel_angular_velocity: float,
) -> Twist:
    """Convert per-side motor power in [-1, 1] to a diff-drive twist."""
    if wheel_separation <= 0.0 or wheel_radius <= 0.0:
        return Twist()

    def wheel_speed(power: float) -> float:
        if not math.isfinite(power):
            return 0.0
        power = max(-1.0, min(1.0, power))
        return power * max_wheel_angular_velocity * wheel_radius

    left_speed = wheel_speed(left)
    right_speed = wheel_speed(right)

    twist = Twist()
    twist.linear.x = (left_speed + right_speed) / 2.0
    twist.angular.z = (right_speed - left_speed) / wheel_separation
    return twist


def apply_motor_deadzone(
    twist: Twist,
    wheel_separation: float,
    deadzone: float,
) -> Twist:
    """Return *twist* with wheel speeds below *deadzone* rounded to zero.

    The diff-drive controller converts the twist to per-side wheel speeds.
    Applying the deadband to those wheel speeds (and converting back) makes
    the simulation behave like motors that stall below a minimum command,
    instead of creeping at arbitrarily small velocities.
    """
    if deadzone <= 0.0 or wheel_separation <= 0.0:
        return twist

    half_separation = wheel_separation / 2.0
    left = twist.linear.x - twist.angular.z * half_separation
    right = twist.linear.x + twist.angular.z * half_separation

    if abs(left) < deadzone:
        left = 0.0
    if abs(right) < deadzone:
        right = 0.0

    result = Twist()
    result.linear.x = (left + right) / 2.0
    result.linear.y = twist.linear.y
    result.linear.z = twist.linear.z
    result.angular.x = twist.angular.x
    result.angular.y = twist.angular.y
    result.angular.z = (right - left) / wheel_separation
    return result


def is_moving(twist: Twist) -> bool:
    """True when a twist commands the drive."""
    return twist.linear.x != 0.0 or twist.angular.z != 0.0


class DrivebaseSimNode(Node):
    """Run the chassis CAN interface on top of the Gazebo diff drive."""

    def __init__(self):
        """Create the CAN, navigation, joystick, and E-stop inputs."""
        super().__init__("drivebase_sim_node")

        self.declare_parameter("wheel_separation", WHEEL_SEPARATION_DEFAULT)
        self.declare_parameter("wheel_radius", WHEEL_RADIUS_DEFAULT)
        self.declare_parameter(
            "max_wheel_angular_velocity",
            MAX_WHEEL_ANGULAR_VELOCITY_DEFAULT)
        self.declare_parameter("motor_deadzone", MOTOR_DEADZONE_DEFAULT)
        self.__wheel_separation = float(
            self.get_parameter("wheel_separation").value)
        self.__wheel_radius = float(
            self.get_parameter("wheel_radius").value)
        self.__max_wheel_angular_velocity = float(
            self.get_parameter("max_wheel_angular_velocity").value)
        self.__motor_deadzone = float(
            self.get_parameter("motor_deadzone").value)

        self.__estop = False

        self.__can_twist = Twist()
        self.__can_rx = float("-inf")
        self.__can_moving_rx = float("-inf")

        self.__nav_twist = Twist()
        self.__nav_rx = float("-inf")

        self.__joystick_twist = Twist()
        self.__joystick_rx = float("-inf")

        self.__publisher = self.create_publisher(
            Twist, OUTPUT_TOPIC, BEST_EFFORT_QOS)
        self.create_subscription(Can, INPUT_TOPIC, self.__on_can, 10)
        self.create_subscription(Twist, NAV_TOPIC, self.__on_nav, 10)
        self.create_subscription(
            Twist, JOYSTICK_TOPIC, self.__on_joystick, 10)
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop_received, 10)
        self.create_timer(PUBLISH_PERIOD_SEC, self.__publish)

        self.get_logger().info(
            f"Drive power from {INPUT_TOPIC} -> {OUTPUT_TOPIC} "
            f"(wheel {self.__wheel_radius:.3f} m, "
            f"{self.__max_wheel_angular_velocity:.1f} rad/s)")

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def __on_can(self, msg: Can):
        """Decode one Drive_Power frame and latch it as the CAN command."""
        try:
            decoded = TeensyCommunication.decode_can_packet(msg)
        except (KeyError, ValueError) as error:
            self.get_logger().warning(
                f"ignoring undecodable Drive_Power frame: {error}")
            return
        if decoded is None:
            return

        twist = drive_power_to_twist(
            decoded.signals["left"].value,
            decoded.signals["right"].value,
            self.__wheel_separation,
            self.__wheel_radius,
            self.__max_wheel_angular_velocity,
        )
        self.__can_twist = apply_motor_deadzone(
            twist, self.__wheel_separation, self.__motor_deadzone)

        now = time.monotonic()
        self.__can_rx = now
        if is_moving(self.__can_twist):
            self.__can_moving_rx = now

    def __on_nav(self, msg: Twist):
        """Latch a navigation velocity command."""
        self.__nav_twist = apply_motor_deadzone(
            msg, self.__wheel_separation, self.__motor_deadzone)
        self.__nav_rx = time.monotonic()

    def __on_joystick(self, msg: Twist):
        """Latch a simulation joystick velocity command."""
        self.__joystick_twist = apply_motor_deadzone(
            msg, self.__wheel_separation, self.__motor_deadzone)
        self.__joystick_rx = time.monotonic()

    def __on_estop_received(self, msg: Bool):
        """Latch or clear the E-stop and stop the simulated drive."""
        estop = bool(msg.data)
        if estop and not self.__estop:
            self.get_logger().info("E-STOP latched: stopping simulated drive")
            self.__publisher.publish(Twist())
        elif not estop and self.__estop:
            self.get_logger().info("E-STOP cleared: drive commands re-enabled")
        self.__estop = estop

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def __selected_twist(self, now: float) -> Twist:
        """Pick the highest-priority fresh command, mirroring the old mux."""
        if self.__estop:
            return Twist()

        can_fresh = now - self.__can_rx <= COMMAND_TIMEOUT_SEC
        if not can_fresh and is_moving(self.__can_twist):
            # The stream died while the drive was still moving.  Drop the
            # latched command instead of driving on stale input.
            self.get_logger().warning(
                f"No Drive_Power frame for {COMMAND_TIMEOUT_SEC:.1f} s; "
                "stopping the simulated drive")
            self.__can_twist = Twist()
            self.__can_moving_rx = now

        if can_fresh and is_moving(self.__can_twist):
            return self.__can_twist

        # A released CAN command keeps the stop active briefly before the
        # lower-priority navigation or joystick source may take over.
        if now - self.__can_moving_rx <= RELEASE_HOLD_SEC:
            return Twist()

        if now - self.__joystick_rx <= MUX_TIMEOUT_SEC:
            return self.__joystick_twist
        if now - self.__nav_rx <= MUX_TIMEOUT_SEC:
            return self.__nav_twist
        return Twist()

    def __publish(self):
        """Publish the selected command to the diff-drive controller."""
        self.__publisher.publish(self.__selected_twist(time.monotonic()))

    def run(self):
        """Spin the node until shutdown."""
        self.get_logger().info("Simulated drivebase hardware started")
        rclpy.spin(self)
        self.get_logger().info("Simulated drivebase hardware stopped")


def main():
    """Create and run the simulated drivebase hardware node."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = DrivebaseSimNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
