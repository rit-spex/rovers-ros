#!/usr/bin/env python3

import time
from typing import Any

# ros imports
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, UInt16

from encoding import BaseStationCommunication, Signal
from command_codes import CONSTANTS
# from constants.CommandCodes import TOPICS_JOYSTICK, TOPICS_BUTTON, CONSTANTS
# from constants.CAN_Constants import CHANNEL, TOPICS

import numpy as np

XBEE_TIMEOUT = 1000000000  # 1,000,000 nano second -> 1 second

class Basestation(Node):
    __basestation_communications: BaseStationCommunication

    __publishers: dict[int, dict[str, rclpy.publisher.Publisher]]

    __subscription: rclpy.subscription.Subscription

    def __init__(self):
        super().__init__("Basestation")

        self.__basestation_communications = BaseStationCommunication()

        self.__publishers = {}

        self.__valueTypes = {
            CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL: Bool,
            CONSTANTS.COMPACT_MESSAGES.UINT_8: Int8,
            CONSTANTS.COMPACT_MESSAGES.UINT_16: UInt16,
            CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK: Float32,
            CONSTANTS.COMPACT_MESSAGES.BOOLEAN: Bool,
        }

        for (id, message) in self.__basestation_communications.get_messages().items():
            self.__publishers[id] = {}
            for (value_name, value_type) in message['values'].items():
                self.__publishers[id][value_name] = self.create_publisher(
                    msg_type=self.__valueTypes[value_type.get_type],
                    topic=f"/BASESTATION/{message['name']}/{value_name}",
                    qos_profile=10
                )

        self.__subscription = self.create_subscription(
            msg_type=UInt8MultiArray,
            topic="/XBEE/MESSAGES",
            callback=self.__on_message_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

    def __on_message_received(self, message: UInt8MultiArray):
        """
        callback function that is called when message is received
        """

        # if start of message is not valid, stop
        # if list(message.data)[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
        #     return

        id: int
        decoded_data: dict[str, Any]

        try:
            # split the message data into a list
            data = list(message.data)
            decoded_data, id = self.__basestation_communications.decode_data(bytes(data[0:]))
        except Exception as e:
            self.get_logger().error(f"Failed to decode message: {e}")
            return

        for (key, value) in decoded_data.items():
            if key not in self.__publishers[id]:
                continue
            
            value_type = self.__valueTypes[
                self.__basestation_communications.get_messages()[id]['values'][key].get_type]

            # self.get_logger().info(f"Publishing to /BASESTATION/{self.__basestation_communications.get_messages()[id]['name']}/{key}: {value}")

            self.__publishers[id][key].publish(value_type(data=value))

    def send_msg(self):
        raise NotImplementedError

    def run(self):
        self.get_logger().info("starting basestation ...")
        rclpy.spin(self)

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, shutting down basestation...")
        rclpy.shutdown()

def main():
    rclpy.init()
    basestation = Basestation()
    basestation.run()


if __name__ == "__main__":
    main()
