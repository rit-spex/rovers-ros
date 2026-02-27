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


class Science(Node):
    __publishers: dict[CAN_MESSAGE_IDS, Publisher]

    def __init__(self):
        super().__init__("science_node")

        self.__publishers = {}
        for (message_id, message) in TEENSY_CAN_MESSAGES.items():
            if(message.subsystem == Subsystems_Names.SCIENCE):
                if not message.isforJetson:
                    self.__publishers[message_id] = self.create_publisher(
                        msg_type=Can,
                        topic=message.topic_name,
                        qos_profile=10
                    )
                else:
                    self.create_subscription(
                        msg_type=Can,
                        topic=message.topic_name,
                        callback=self.__on_can_message_received,
                        qos_profile=10,
                    )

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

    def __on_can_message_received(self, msg: Can):
        
        self.get_logger().info(f"Received CAN message on topic {msg.topic}")

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping science..")
        rclpy.shutdown()

    def run(self):
        self.get_logger().info("starting science...")
        rclpy.spin(self)
        self.get_logger().info("stopping science...")


def main():
    rclpy.init()
    science = Science()
    science.run()


if __name__ == "__main__":
    main()
