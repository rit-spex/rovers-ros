#!/usr/bin/env python3

"""Publish the simulated rover's build to the basestation.

The real rover learns which subsystems are present from Teensy heartbeats
and the master node.  Gazebo has no Teensy, so this node supplies the same
``subsystem_enabled`` telemetry fields from a launch argument:

    drivebase       drivebase only
    arm_drivebase   drivebase + arm
    life_drivebase  drivebase + life detection

The basestation uses those fields to show only the options for the
selected build.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

CONFIG_MODULES = {
    "drivebase": (False, False),
    "arm_drivebase": (True, False),
    "life_drivebase": (False, True),
}
DEFAULT_CONFIG = "arm_drivebase"


class SimConfig(Node):
    """Publish simulated subsystem presence for the basestation UI."""

    def __init__(self):
        super().__init__("sim_config")

        self.declare_parameter("config", DEFAULT_CONFIG)
        config = self.get_parameter(
            "config").get_parameter_value().string_value
        if config not in CONFIG_MODULES:
            self.get_logger().warning(
                f"Unknown robot config '{config}'; using {DEFAULT_CONFIG}")
            config = DEFAULT_CONFIG
        self.__config = config
        arm_enabled, life_enabled = CONFIG_MODULES[config]

        self.__arm_publisher = self.create_publisher(
            Bool, "/ARM/STATE/ENABLED", 10)
        self.__life_publisher = self.create_publisher(
            Bool, "/ROVER/TELEMETRY/SUBSYSTEM/LIFE_ENABLED", 10)
        self.__auto_publisher = self.create_publisher(
            Bool, "/ROVER/TELEMETRY/SUBSYSTEM/AUTO_ENABLED", 10)

        self.__arm_enabled = Bool(data=arm_enabled)
        self.__life_enabled = Bool(data=life_enabled)
        self.__auto_enabled = Bool(data=False)

        self.create_timer(1.0, self.__publish)
        self.get_logger().info(f"Simulated rover config: {config}")

    def __publish(self):
        self.__arm_publisher.publish(self.__arm_enabled)
        self.__life_publisher.publish(self.__life_enabled)
        self.__auto_publisher.publish(self.__auto_enabled)


def main():
    rclpy.init()
    node = SimConfig()
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
