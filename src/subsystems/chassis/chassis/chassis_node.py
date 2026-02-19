from typing import Any
from constants.CAN_Constants import TOPICS
from constants.CommandCodes import CONSTANTS

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import Can
from std_msgs.msg import Float32
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16


class Chassis(Node):
    __topic: dict[str, Any]
    __can_publisher: Publisher

    __LY_value: float
    __RX_value: float

    def __init__(self):
        super().__init__("chassis_node")

        self.__topic = TOPICS[3]
        self.__can_publisher = self.create_publisher(
            msg_type=Can,
            topic=f"/CAN/TX/{self.__topic['name']}",
            qos_profile=10,
        )

        self.create_subscription(
            Float32, "/BASESTATION/" + CONSTANTS.XBOX.NAME + "/" + CONSTANTS.XBOX.JOYSTICK.AXIS_LY_STR, self.__LY_callback, 10
        )
        self.create_subscription(
            Float32, "/BASESTATION/" + CONSTANTS.XBOX.NAME + "/" + CONSTANTS.XBOX.JOYSTICK.AXIS_RY_STR, self.__RY_callback, 10
        )
        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.__LY_value = 0
        self.__RX_value = 0

    def __LY_callback(self, msg: Float32):
        self.__LY_value = msg.data
        self.__send_controller_data()
        # self.get_logger().info(f"LY_callback message: {msg}")
        # self.get_logger().info(f"LY_value is now {self.__LY_value}")

    def __RY_callback(self, msg: Float32):
        self.__RX_value = msg.data
        self.__send_controller_data()
        # self.get_logger().info(f"RX_callback message: {msg}")
        # self.get_logger().info(f"RX_value is now {self.__RX_value}")

    def __send_controller_data(self):
        # self.get_logger().info(f"LY: {self.__LY_value}")
        # self.get_logger().info(f"RX: {self.__RX_value}")
        # self.get_logger().info(f"")

        ros_msg = Can()
        ros_msg.channel = self.__topic["channel"]
        ros_msg.id = self.__topic["id"]
        ros_msg.buf = [
            int((self.__LY_value * 100) + 100),
            int((self.__RX_value * 100) + 100),
            0,
            0,
            0,
            0,
            0,
            0,
        ]

        self.__can_publisher.publish(ros_msg)

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping chassis...")
        self.__LY_value = 0
        self.__RX_value = 0
        self.__send_controller_data()
        rclpy.shutdown()

    def run(self):
        self.get_logger().info("starting chassis...")
        rclpy.spin(self)
        self.get_logger().info("stopping chassis")


def main():
    rclpy.init()
    chassis = Chassis()
    try:
        chassis.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        chassis.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
