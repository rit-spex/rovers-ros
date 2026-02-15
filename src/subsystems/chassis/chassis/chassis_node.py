from typing import Any
from constants.CAN_Constants import TOPICS
from constants.CommandCodes import CONSTANTS

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import Can
from std_msgs.msg import Float32
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16


class Chassis(Node):
    __topic: dict[str, Any]
    __can_publisher: Publisher

    __LY_value: float
    __RY_value: float
    # 0-manual  1-object_detection  2-arcu_tracking  3-GPS  4-ARM_keyboard
    __operation_type = 0

    def __init__(self):
        super().__init__("chassis_node")

        self.__topic = TOPICS[3]
        self.__can_publisher = self.create_publisher(
            msg_type=Can,
            topic=f"/CAN/TX/{self.__topic['name']}",
            qos_profile=10,
        )

        # From controller (manual driving)
        self.create_subscription(
            Float32, "/BASESTATION/" + CONSTANTS.XBOX.NAME + "/" + CONSTANTS.XBOX.JOYSTICK.AXIS_LY_STR, self.__LY_manual_callback, 10
        )
        self.create_subscription(
            Float32, "/BASESTATION/" + CONSTANTS.XBOX.NAME + "/" + CONSTANTS.XBOX.JOYSTICK.AXIS_RY_STR, self.__RY_manual_callback, 10
        )

        # From Object Detection
        self.create_subscription(
            Float32, "/object_detection/OD_LY", self.__OD_LY_callback, 10
        )
        self.create_subscription(
            Float32, "/object_detection/OD_RY", self.__OD_RY_callback, 10
        )

        # From Arcu Detection
        self.create_subscription(
            Float32, "/object_detection/AR_LY", self.__AR_LY_callback, 10
        )
        self.create_subscription(
            Float32, "/object_detection/AR_RY", self.__AR_RY_callback, 10
        )

        self.create_subscription(
            Int8, "/BASESTATION/XBOX/ROBOT_MODE", self.__operation_type_callback, 10
        )

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.__LY_value = 0
        self.__RY_value = 0

    def __operation_type_callback(self, msg: Int8):
        self.__operation_type = msg.data

    def __LY_manual_callback(self, msg: Float32):
        if self.__operation_type == 0:
            self.__LY_value = msg.data
            self.__send_controller_data()
            # self.get_logger().info(f"LY_callback message: {msg}")
            # self.get_logger().info(f"LY_value is now {self.__LY_value}")

    def __RY_manual_callback(self, msg: Float32):
        if self.__operation_type == 0:
            self.__RY_value = msg.data
            self.__send_controller_data()
            # self.get_logger().info(f"RX_callback message: {msg}")
            # self.get_logger().info(f"RX_value is now {self.__RX_value}")

    def __OD_LY_callback(self, msg: Float32):
        if self.__operation_type == 1:
            self.__LY_value = msg.data
            self.__send_controller_data()

    def __OD_RY_callback(self, msg: Float32):
        if self.__operation_type == 1:
            self.__RY_value = msg.data
            self.__send_controller_data()

    def __AR_LY_callback(self, msg: Float32):
        if self.__operation_type == 2:
            self.__LY_value = msg.data
            self.__send_controller_data()

    def __AR_RY_callback(self, msg: Float32):
        if self.__operation_type == 2:
            self.__RY_value = msg.data
            self.__send_controller_data()  

    def __send_controller_data(self):
        # self.get_logger().info(f"LY: {self.__LY_value}")
        # self.get_logger().info(f"RX: {self.__RX_value}")
        # self.get_logger().info(f"")

        ros_msg = Can()
        ros_msg.channel = self.__topic["channel"]
        ros_msg.id = self.__topic["id"]
        ros_msg.buf = [
            int((self.__LY_value * 100) + 100),
            int((self.__RY_value * 100) + 100),
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
    chassis.run()


if __name__ == "__main__":
    main()
