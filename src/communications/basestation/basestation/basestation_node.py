#!/usr/bin/env python3

from typing import Any

import time

# ros imports
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.publisher import Publisher
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, Float32

import numpy as np

XBEE_TIMEOUT = 1000000000  # 1,000,000 nano second -> 1 second


class Basestation(Node):
    # flag to determine if the xbee should be disabled based on no signal
    __disabled: bool

    # flag to be triggered once the xbee has received any data
    __is_first_connected: bool

    def __init__(self):
        super().__init__("Basestation_node")

        self.__disabled = False
        self.__is_first_connected = False

        # set all of the topics to default state

        # creates publishers for all the different buttons
        self.__joystick_publishers: dict[int, Publisher] = {}
        for joystick in CONSTANTS.XBOX.JOYSTICK.LIST_OF_AXIS:
            self.__joystick_publishers[joystick] = self.create_publisher(
                TOPICS_JOYSTICK[joystick]["val"],
                f"/Xbee/RX/Xbox/Axis/{TOPICS_JOYSTICK[joystick]['name']}",
                10,
            )
            self.__name_to_ID_XBOX[TOPICS_JOYSTICK[joystick]["name"]] = joystick
            self.__value_state["xbox"][
                TOPICS_JOYSTICK[joystick]["name"]
            ] = CONSTANTS.XBOX.JOYSTICK.NEUTRAL_HEX
        self.__button_publishers: dict[int, Publisher] = {}
        for button in CONSTANTS.XBOX.BUTTONS.LIST_OF_BUTTONS:
            self.__button_publishers[button] = self.create_publisher(
                TOPICS_BUTTON[button]["val"],
                f"/Xbee/RX/Xbox/Buttons/{TOPICS_BUTTON[button]['name']}",
                10,
            )
            self.__name_to_ID_XBOX[TOPICS_BUTTON[button]["name"]] = button
            self.__value_state["xbox"][TOPICS_BUTTON[button]["name"]] = 0

        self.__n64_publishers = {
            "A": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/A", 10),
            "B": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/B", 10),
            "L": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/L", 10),
            "R": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/R", 10),
            "CU": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/CU", 10),
            "CD": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/CD", 10),
            "CL": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/CL", 10),
            "CR": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/CR", 10),
            "DU": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/DU", 10),
            "DD": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/DD", 10),
            "DL": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/DL", 10),
            "DR": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/DR", 10),
            "Z": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/Z", 10),
        }

        self.__value_state["n64"] = {
            "A": CONSTANTS.N64.BUTTONS.OFF,
            "B": CONSTANTS.N64.BUTTONS.OFF,
            "L": CONSTANTS.N64.BUTTONS.OFF,
            "R": CONSTANTS.N64.BUTTONS.OFF,
            "CU": CONSTANTS.N64.BUTTONS.OFF,
            "CD": CONSTANTS.N64.BUTTONS.OFF,
            "CL": CONSTANTS.N64.BUTTONS.OFF,
            "CR": CONSTANTS.N64.BUTTONS.OFF,
            "DU": CONSTANTS.N64.BUTTONS.OFF,
            "DD": CONSTANTS.N64.BUTTONS.OFF,
            "DL": CONSTANTS.N64.BUTTONS.OFF,
            "DR": CONSTANTS.N64.BUTTONS.OFF,
            "Z": CONSTANTS.N64.BUTTONS.OFF,
        }
    
    def __publish_new_controls(
        self, controller: str, button: str, value: Float32 | Bool
    ) -> None:
        """Helper function to only publish updated information

        Args:
            controller (str): the name of the controller this value is for ('xbox', 'n64')
            button (str): the name of the button this value this for
            value (Any): the value to publish
        """

        if self.__value_state[controller][button] == value:
            return
        self.__value_state[controller][button] = value

        if controller == "n64":
            self.__n64_publishers[button].publish(value)
            return

        if button.upper() in ["LY", "LX", "RY", "RX"]:
            self.__joystick_publishers[self.__name_to_ID_XBOX[button]].publish(value)
            return
        self.__button_publishers[self.__name_to_ID_XBOX[button]].publish(value)

    def __parse_incoming_message(self, message: list[int]):
        """
        Helper function that is called when message is received, to parse to get values

        Args:
            message (list[int]): a full message that start with start message
        """

    def send_msg(self):
        raise NotImplementedError

    def on_message_received(self, var1):
        """
        callback function that is called when message is received
        """

    def run(self):
        # last_cycle_time = time.time_ns()
        # self.__xbee_device.add_data_received_callback(self.on_message_received)
        rclpy.spin(self)
        # last_cycle_time = time.time_ns()

        # self.get_logger().info("starting xbee...")

        # while not self.__disabled:
        #     if time.time_ns() - last_cycle_time > XBEE_UPDATE_RATE:
        #         last_cycle_time = time.time_ns()

        # Signalling E-STOP
        # ros_msg = Can()
        # ros_msg.id = 0
        # ros_msg.channel = CHANNEL.MAIN_BODY
        # ros_msg.buf = [0, 0, 0, 0, 0, 0, 0, 0]
        # self.create_publisher(Can, "/CAN/TX/E_STOP", 10).publish(ros_msg)


def main():
    rclpy.init()
    basestation = Basestation()
    basestation.run()

if __name__ == "__main__":
    main()
