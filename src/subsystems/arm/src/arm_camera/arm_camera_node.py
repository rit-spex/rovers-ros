"""Arm camera node placeholder."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ArmCameraNode(Node):
    """Placeholder for the arm camera node."""

    def __init__(self):
        super().__init__("arm_camera_node")


def main():
    """Run the arm camera node."""
    rclpy.init()
    node = ArmCameraNode()
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
