"""Normalize arm teleop input into controller commands.

This node owns the SpaceMouse input semantics: axis signs, deadzone, power
curve, sensitivity, and latching.  It publishes normalized teleop intent so
``arm_controller_node`` never has to know which device produced it.

Current outputs:

* ``/ARM/CMD_VELOCITY`` (``custom_interfaces/ArmVelocity``) - end-effector
  linear velocity [m/s] and angular velocity [rad/s] in the arm base frame.
* ``/ARM/CMD_GRIPPER_AXIS`` (``std_msgs/Float32``) - processed gripper axis
  in [-1, 1], used only while the controller is in gripper-drive mode.

Autonomy and other non-teleop sources should feed the same command topics
through a separate input/mux node.  ``custom_interfaces/ArmPose`` is defined
for absolute target poses.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from custom_interfaces.msg import ArmVelocity
from std_msgs.msg import Float32, Int16

from constants import (
    AXIS_SIGNS,
    DEADZONE,
    DT,
    GRIPPER_AXIS_TOPIC,
    INPUT_EXPONENT,
    METERS_PER_INCH,
    RAW_MAX,
    ROTATION_SENS,
    TRANSLATION_SENS,
    VELOCITY_TOPIC,
)

DEFAULT_PUBLISH_RATE_HZ = 50.0
TELEOP_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# Raw SpaceMouse axis topics published by the basestation bridge.
SPACEMOUSE_AXIS_TOPICS = {
    "x": "/BASESTATION/spacemouse/x",
    "y": "/BASESTATION/spacemouse/y",
    "z": "/BASESTATION/spacemouse/z",
    "rx": "/BASESTATION/spacemouse/rx",
    "ry": "/BASESTATION/spacemouse/ry",
    "rz": "/BASESTATION/spacemouse/rz",
}


def process_axis(raw_value: float) -> float:
    """Normalize, deadzone, and power-curve one raw SpaceMouse axis."""
    normalized = max(-1.0, min(1.0, raw_value / RAW_MAX))
    if abs(normalized) < DEADZONE:
        return 0.0
    sign = 1.0 if normalized > 0 else -1.0
    rescaled = (abs(normalized) - DEADZONE) / (1.0 - DEADZONE)
    return sign * (rescaled ** INPUT_EXPONENT)


class ArmTeleopNode(Node):
    """Turn raw operator axes into normalized arm commands."""

    def __init__(self):
        """Create the axis subscriptions, publishers, and output timer."""
        super().__init__("arm_teleop_node")

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)

        rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value))

        # Latched raw axis values.  The basestation only publishes on change,
        # so the node republishes the normalized command continuously.
        self.__axes = dict.fromkeys(SPACEMOUSE_AXIS_TOPICS, 0)

        self.__velocity_publisher = self.create_publisher(
            msg_type=ArmVelocity,
            topic=VELOCITY_TOPIC,
            qos_profile=10,
        )
        self.__gripper_publisher = self.create_publisher(
            msg_type=Float32,
            topic=GRIPPER_AXIS_TOPIC,
            qos_profile=10,
        )

        for axis, topic in SPACEMOUSE_AXIS_TOPICS.items():
            self.create_subscription(
                msg_type=Int16,
                topic=topic,
                callback=self.__make_axis_callback(axis),
                qos_profile=TELEOP_QOS,
            )

        self.create_timer(1.0 / rate_hz, self.__publish_commands)

    def __make_axis_callback(self, axis: str):
        """Build a callback that latches one raw axis value."""
        def callback(msg: Int16):
            self.__axes[axis] = int(msg.data)
        return callback

    def __axis_velocity(self, axis: str, sensitivity: float) -> float:
        """Convert one latched axis to a controller-frame velocity."""
        scaled = process_axis(self.__axes[axis] * AXIS_SIGNS[axis])
        return scaled * sensitivity / DT

    def __publish_commands(self):
        """Publish the latest normalized velocity and gripper axis."""
        velocity = ArmVelocity()

        # TRANSLATION_SENS works in inches per control cycle; the ROS
        # command is m/s, so convert at this boundary.
        velocity.linear_x = (
            self.__axis_velocity("x", TRANSLATION_SENS) * METERS_PER_INCH)
        velocity.linear_y = (
            self.__axis_velocity("y", TRANSLATION_SENS) * METERS_PER_INCH)
        velocity.linear_z = (
            self.__axis_velocity("z", TRANSLATION_SENS) * METERS_PER_INCH)
        velocity.angular_x = self.__axis_velocity("rx", ROTATION_SENS)
        velocity.angular_y = self.__axis_velocity("ry", ROTATION_SENS)
        velocity.angular_z = self.__axis_velocity("rz", ROTATION_SENS)

        self.__velocity_publisher.publish(velocity)
        self.__gripper_publisher.publish(
            Float32(data=process_axis(self.__axes["ry"] * AXIS_SIGNS["ry"])))

    def run(self):
        """Spin the node until it is shut down."""
        self.get_logger().info("Arm teleop started")
        rclpy.spin(self)
        self.get_logger().info("Arm teleop stopped")


def main():
    """Run the arm teleop node."""
    rclpy.init()
    node = None
    try:
        node = ArmTeleopNode()
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
