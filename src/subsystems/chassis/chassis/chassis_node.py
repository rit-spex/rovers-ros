from typing import Any
from constants.CommandCodes import CONSTANTS
from constants.CAN_structs import Message
from constants.CAN_constants import CAN_MESSAGE_IDS, TEENSY_CAN_MESSAGES, Subsystems_Names
from constants.can_encoding import TeensyCommunication

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import Can
from std_msgs.msg import Float32
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16


class Chassis(Node):
    __topic: dict[str, Any]

    __LY_value: float
    __RY_value: float

    __publishers: dict[CAN_MESSAGE_IDS, Publisher]

    __drive_power_can_message: Message

    def __init__(self):
        super().__init__("chassis_node")

        self.__publishers = {}
        for (message_id, message) in TEENSY_CAN_MESSAGES.items():
            if(message.subsystem == Subsystems_Names.CHASSIS):
                self.__publishers[message_id] = self.create_publisher(
                    msg_type=Can,
                    topic=message.topic_name,
                    qos_profile=10
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
        self.__RY_value = 0
        self.__drive_power_can_message = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.DRIVE_POWER]

    def __LY_callback(self, msg: Float32):
        self.__LY_value = msg.data
        self.__drive_power_can_message.signals["left"].set_value(self.__LY_value)
        can_packet = TeensyCommunication.encode_can_message(self.__drive_power_can_message)
        self.__publishers[CAN_MESSAGE_IDS.DRIVE_POWER].publish(can_packet)
        self.get_logger().info(f"LY_callback message: {msg}")
        self.get_logger().info(f"Topic is {self.__drive_power_can_message.topic_name}")

    def __RY_callback(self, msg: Float32):
        self.__RY_value = msg.data
        self.__drive_power_can_message.signals["right"].set_value(self.__RY_value)
        can_packet = TeensyCommunication.encode_can_message(self.__drive_power_can_message)
        self.__publishers[CAN_MESSAGE_IDS.DRIVE_POWER].publish(can_packet)
        # self.get_logger().info(f"RY_callback message: {msg}")
        # self.get_logger().info(f"Topic is {self.__drive_power_can_message.topic_name}")

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping chassis...")
        # self.__LY_value = 0
        # self.__RY_value = 0
        # self.__LY_Publisher.publish(self.__LY_value)
        # self.__RY_Publisher.publish(self.__RY_value)

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
