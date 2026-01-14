#!/usr/bin/env python3

import time
from typing import Any

# ros imports
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, Float32, UInt8MultiArray, UInt8, UInt16, UInt32
from custom_interfaces.msg import CanFD, Can
from communications.teensy_comms.teensy_comms.can_encoding import TeensyCommunication
# from constants.CommandCodes import TOPICS_JOYSTICK, TOPICS_BUTTON, CONSTANTS
from constants.CAN_Constants import TEENSY_CAN_MESSAGES, DATA_TYPES

import numpy as np

class Teensy(Node):
    __teensy_communications: TeensyCommunication

    __publishers: dict[int, dict[str, rclpy.publisher.Publisher]]

    # Publish message to the teensy by sending to CAN nodes
    __CAN_publisher: rclpy.publisher.Publisher


    def __init__(self):
        super().__init__("Teensy")

        self.__teensy_communications = TeensyCommunication()

        # store all publishers based on message and signal names for later reference
        self.__publishers = {}
        self.__subscribers = {}

        # Map data types to ROS message types
        self.__valueTypes = {
            DATA_TYPES.UINT_8: UInt8,
            DATA_TYPES.UINT_16: UInt16,
            DATA_TYPES.UINT_32: UInt32
        }

        # Create subscription for E-STOP messages to end the node
        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        # ******************************************************************************
        # ****************************    Teensy->Jetson   *****************************
        # ******************************************************************************

        # Create publishers for each message that is sent to the Jetson
        for (id, message) in TEENSY_CAN_MESSAGES.items():
            # Only create publishers for messages going to the Jetson
            if(not message.isforJetson):
                continue

            self.__publishers[id] = {}

            # Create publishers for each signal in the message
            for (signal_name, signal) in message.signals.items():
                self.__publishers[id][signal_name] = self.create_publisher(
                    msg_type=self.__valueTypes[signal.type],
                    topic=signal.topic_src,
                    qos_profile=10
                )

        self.create_subscription(
            msg_type=Can,
            topic="CAN/RX",
            callback=self.__on_new_jetson_message,
            qos_profile=10,
        )

        # ******************************************************************************
        # ****************************    Jetson->Teensy   *****************************
        # ******************************************************************************

        # Create subscribers for each message that is sent for the Teensy
        for (id, message) in TEENSY_CAN_MESSAGES.items():
            # Only create subscribers for messages going to the Teensy
            if(message.isforJetson):
                continue

            # Create subscribers for each signal in the message
            for (signal_name, signal) in message.signals.items():
                self.__subscribers[id][signal_name] = self.create_subscription(
                    msg_type=self.__valueTypes[signal.type],
                    topic=signal.topic_src,
                    callback=lambda msg, id=id: self.__on_signal_to_send(msg, id),
                    qos_profile=10
                )

        self.__CAN_publisher = self.create_publisher(
            msg_type=Can,
            topic="CAN/TX",
            qos_profile=10,
        )

    def __on_signal_to_send(self, msg: Any, id: int):
        self.get_logger().info("Signal to send received")
        # Process the signal to send

    def __on_new_jetson_message(self, msg: Can):
        """
        callback function when a new message for the Jetson is received
        """
        self.get_logger().info("New Jetson message received")

        decoded_data: dict[str, Any]

        try:
            # split the message data into a list
            decoded_data = self.__basestation_communications.decode_data(bytes(data[0:]))
        except Exception as e:
            self.get_logger().error(f"Failed to decode message: {e}")
            return

        for (key, value) in decoded_data.items():
            if key not in self.__publishers[id]:
                continue
            
            value_type = self.__valueTypes[TEENSY_CAN_MESSAGES[id].signals[key].type]

            self.__publishers[id][key].publish(value_type(data=value))

    def run(self):
        self.get_logger().info("starting teensy_comms ...")
        rclpy.spin(self)

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, shutting down teensy_comms...")
        rclpy.shutdown()

def main():
    rclpy.init()
    teensy_comms = Teensy()
    teensy_comms.run()


if __name__ == "__main__":
    main()
