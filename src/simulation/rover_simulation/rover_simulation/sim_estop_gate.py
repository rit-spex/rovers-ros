"""Keep the simulated diff drive stopped while /ESTOP is active.

Gazebo's diff drive is commanded through twist_mux, separately from the real
rover's CAN path. This node publishes a zero Twist on twist_mux's highest
priority input so navigation or joystick teleop cannot move the simulated
rover during an E-STOP.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

ESTOP_CMD_TOPIC = "/GAZEBO/CHASSIS/diff_cont/cmd_vel_estop"


class SimEstopGate(Node):
    """Publish zero velocity while /ESTOP is latched."""

    def __init__(self):
        super().__init__("sim_estop_gate")
        self.__estop = False

        self.__publisher = self.create_publisher(Twist, ESTOP_CMD_TOPIC, 10)
        self.create_subscription(Bool, "/ESTOP", self.__on_estop_received, 10)
        self.create_timer(0.1, self.__publish)

    def __on_estop_received(self, msg: Bool):
        self.__estop = bool(msg.data)

    def __publish(self):
        if self.__estop:
            self.__publisher.publish(Twist())


def main():
    rclpy.init()
    node = SimEstopGate()
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
