"""Drive the Gazebo drivebase from the rover's CAN interface.

The real rover reaches its motors through exactly one path: ``chassis_node``
turns basestation joystick axes into a ``Drive_Power`` frame on
``CAN/TX/CHASSIS/Drive_Power``, and the Teensy turns that frame into motor
power.  Gazebo has no Teensy, so this node stands in for it: it subscribes to
the same topic, decodes it with the shared protocol, and feeds the result to
the simulated diff-drive controller through ``twist_mux``.

``Drive_Power`` carries normalised motor power per side in ``[-1, 1]``, while
Gazebo wants wheel speed.  Power is therefore scaled by
``max_wheel_angular_velocity`` (rad/s) and the wheel radius.  The rest of the
simulation stack (``twist_mux`` -> ``cmd_vel_bridge`` -> ``diff_cont``) is
unchanged, so E-STOP gating, navigation priority, and the drive-motor
deadzone all keep working.

The CAN input only takes over while it is actually commanding motion.  It
publishes at a fixed rate so ``twist_mux`` does not time the input out between
joystick frames, then stops as soon as the sticks are released, so
``/cmd_vel`` navigation and the direct joystick path still reach the drive
when nobody is driving from the basestation.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from constants.CAN_constants import CAN_MESSAGE_IDS, TEENSY_CAN_MESSAGES
from constants.can_encoding import TeensyCommunication
from custom_interfaces.msg import Can

from rover_simulation.cmd_vel_bridge import (
    MOTOR_DEADZONE_DEFAULT,
    apply_motor_deadzone,
)

# Same frame chassis_node publishes on the real rover.
INPUT_TOPIC = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.DRIVE_POWER].topic_name
OUTPUT_TOPIC = "/GAZEBO/CHASSIS/diff_cont/cmd_vel_can"

PUBLISH_PERIOD_SEC = 0.1
# Keep publishing a released command briefly so a stop is not lost while the
# mux hands the drive back to another input.
RELEASE_HOLD_SEC = 0.3
# Drive_Power frames are streamed by the basestation.  Silence this long means
# the command path is gone, so stop treating the latched command as current.
COMMAND_TIMEOUT_SEC = 1.0
# twist_mux subscribes with the default profile, which is reliable.
RELIABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

# Keep in sync with config/my_controllers.yaml.
WHEEL_SEPARATION_DEFAULT = 0.58
WHEEL_RADIUS_DEFAULT = 0.081674
# Matches the wheel velocity command limit in ros2_control.xacro.
MAX_WHEEL_ANGULAR_VELOCITY_DEFAULT = 10.0


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


def is_moving(twist: Twist) -> bool:
    """True when a twist commands the drive once the motor deadzone applies."""
    return twist.linear.x != 0.0 or twist.angular.z != 0.0


class CanDriveBridge(Node):
    """Republish the rover's Drive_Power CAN command as a diff-drive twist."""

    def __init__(self):
        super().__init__("can_drive_bridge")

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

        self.__twist = Twist()
        # -inf so the node stays passive until a nonzero command arrives,
        # including at startup before the first frame.
        self.__last_moving_rx = float("-inf")
        self.__last_rx = float("-inf")

        self.__publisher = self.create_publisher(
            Twist, OUTPUT_TOPIC, RELIABLE_QOS)
        self.create_subscription(Can, INPUT_TOPIC, self.__on_can, 10)
        self.create_timer(PUBLISH_PERIOD_SEC, self.__publish)

        self.get_logger().info(
            f"Drive power from {INPUT_TOPIC} -> {OUTPUT_TOPIC} "
            f"(wheel {self.__wheel_radius:.3f} m, "
            f"{self.__max_wheel_angular_velocity:.1f} rad/s)")

    def __on_can(self, msg: Can):
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
        self.__twist = apply_motor_deadzone(
            twist, self.__wheel_separation, self.__motor_deadzone)

        now = time.monotonic()
        self.__last_rx = now
        if is_moving(self.__twist):
            self.__last_moving_rx = now

    def __publish(self):
        now = time.monotonic()
        stale = now - self.__last_rx > COMMAND_TIMEOUT_SEC
        if stale and is_moving(self.__twist):
            # The command stream died while the drive was still moving.
            # Drop the latched command and emit one stop through the release
            # hold instead of driving on a command nobody is refreshing.
            self.get_logger().warning(
                f"No Drive_Power frame for {COMMAND_TIMEOUT_SEC:.1f} s; "
                "stopping the simulated drive")
            self.__twist = Twist()
            self.__last_moving_rx = now

        if not stale and is_moving(self.__twist):
            self.__publisher.publish(self.__twist)
        elif now - self.__last_moving_rx <= RELEASE_HOLD_SEC:
            # Finish the stop, then go passive so lower-priority inputs win.
            self.__publisher.publish(Twist())


def main():
    rclpy.init()
    node = CanDriveBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
