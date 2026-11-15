"""Bridge twist_mux output to the Gazebo diff-drive controller.

``twist_mux`` publishes with RELIABLE QoS, while ``diff_drive_controller``
subscribes to ``cmd_vel_unstamped`` with BEST_EFFORT QoS.  Those profiles are
incompatible, so the controller never receives mux output and keeps the last
nonzero wheel velocity.  This relay matches both sides and publishes zero
whenever the mux goes quiet, so the simulated rover cannot creep.

A small per-wheel deadzone models the real drive motors: a wheel speed
command below ``motor_deadzone`` m/s is rounded to zero.  Tiny joystick or
navigation commands therefore do not move the simulated rover, matching the
real hardware's motor deadband.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

INPUT_TOPIC = "/GAZEBO/CHASSIS/diff_cont/cmd_vel_mux"
OUTPUT_TOPIC = "/GAZEBO/CHASSIS/diff_cont/cmd_vel_unstamped"
ZERO_TIMEOUT_SEC = 0.25
PUBLISH_PERIOD_SEC = 0.1

# Keep in sync with wheel_separation in config/my_controllers.yaml.
WHEEL_SEPARATION_DEFAULT = 0.58
# Small drive-motor deadband at the wheel [m/s].
MOTOR_DEADZONE_DEFAULT = 0.1

RELIABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
BEST_EFFORT_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


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


class CmdVelBridge(Node):
    """Relay mux commands and force a zero command when the mux is silent."""

    def __init__(self):
        super().__init__("cmd_vel_bridge")

        self.declare_parameter(
            "wheel_separation", WHEEL_SEPARATION_DEFAULT)
        self.__wheel_separation = float(
            self.get_parameter("wheel_separation").value)
        self.declare_parameter(
            "motor_deadzone", MOTOR_DEADZONE_DEFAULT)
        self.__motor_deadzone = float(
            self.get_parameter("motor_deadzone").value)

        self.__publisher = self.create_publisher(
            Twist, OUTPUT_TOPIC, BEST_EFFORT_QOS)
        self.create_subscription(
            Twist, INPUT_TOPIC, self.__on_command, RELIABLE_QOS)
        self.__last = Twist()
        self.__last_rx = 0.0
        self.create_timer(PUBLISH_PERIOD_SEC, self.__publish)

        self.get_logger().info(
            f"Drive motor deadzone {self.__motor_deadzone:.3f} m/s "
            f"(wheel separation {self.__wheel_separation:.2f} m)")

    def __on_command(self, msg: Twist):
        self.__last = msg
        self.__last_rx = time.monotonic()

    def __publish(self):
        if time.monotonic() - self.__last_rx > ZERO_TIMEOUT_SEC:
            self.__publisher.publish(Twist())
            return

        self.__publisher.publish(
            apply_motor_deadzone(
                self.__last,
                self.__wheel_separation,
                self.__motor_deadzone,
            )
        )


def main():
    rclpy.init()
    node = CmdVelBridge()
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
