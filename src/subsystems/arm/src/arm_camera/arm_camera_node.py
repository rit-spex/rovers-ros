"""Arm camera node placeholder."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node


class ArmCameraNode(Node):
    """Placeholder for the arm camera node."""

    def __init__(self):
        super().__init__("arm_camera_node")


def main():
    """Run the arm camera node."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = ArmCameraNode()
        rclpy.spin(node)
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
