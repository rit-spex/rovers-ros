#!/usr/bin/env python3

from typing import Any
from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import TimeoutException

# import constants.CommanndCodes
from constants.CommanndCodes import CONSTANTS

import time

# ros imports
import rclpy
import rclpy.logging
from rclpy.node import Node
# from rclpy.publisher import Publisher

# from custom_interfaces.msg import CanFD, Can
from constants.CommanndCodes import TOPICS_JOYSTICK, TOPICS_BUTTON
import rclpy.publisher
import rclpy.subscription
# import xbee
# import xbee.xbee
# import xbee.xbee.xbee_node

XBEE_PORT = "/dev/ttyUSB0"
XBEE_SPEED = 921600
XBEE_UPDATE_RATE = 40000000  # 40000 nano second -> 40 micro second
XBEE_TIMEOUT = 1000000000  # 1,000,000 nano second -> 1 second


class Xbee(Node):
    # flag to determine if the xbee should be disabled based on no signal
    __disabled: bool

    # flag to be triggered once the xbee has received any data
    __is_first_connected: bool

    # all the current values from the xbee
    # self.__button_values = [False] * CONSTANTS.NUM_BUTTONS
    __axis_values: list[float]

    # open the port to the device
    __xbee_device: XBeeDevice

    # track when the last successful message was received
    __last_successful_message: int

    def __init__(self):
        super().__init__("Xbee_node")

        self.__disabled = False
        self.__is_first_connected = False
        self.__axis_values = [0.0] * CONSTANTS.NUM_AXES
        self.__xbee_device = XBeeDevice(XBEE_PORT, XBEE_SPEED)
        self.__xbee_device.open()
        self.__last_successful_message = time.time_ns()

    def __del__(self):
        # close device on deletion
        self.__xbee_device.close()

    def is_disabled(self) -> bool:
        return self.__disabled

    def set_disabled(self, disabled: bool):
        self.__disabled = disabled

    def __parse_incoming_message(self, message: list[int]):
        """
        helper function that is called when message is received, to parse to get values

        :param message - a full message that start with start message
        """

        # the current byte number
        byte_num: int = 0

        # parse for axis
        for i in range(0, CONSTANTS.NUM_USED_AXES, 1):
            if (
                CONSTANTS.JOYSTICK.MAX_VALUE
                >= message[byte_num]
                >= CONSTANTS.JOYSTICK.MIN_VALUE
            ):
                self.__axis_values[i] = (message[byte_num] - 100.0) / (100.0)
                byte_num = byte_num + 1

        # parse for button values
        for i in range(0, CONSTANTS.NUM_BUTTONS, 1):
            if i != 0 and i % 4 == 0:
                byte_num = byte_num + 1

            # check if section of byte is on or off
            print(message[byte_num])

            button_value = (
                (
                    message[byte_num]
                    // pow(
                        2,
                        (i % CONSTANTS.BUTTONS.NUM_BUTTONS_PER_BYTE)
                        * CONSTANTS.BUTTONS.SIZE_BUTTON_IN_BITS,
                    )
                )
                % 4
            ) == CONSTANTS.BUTTONS.ON

            self.create_publisher(
                TOPICS_BUTTON[i]["val"],
                f"/Xbee/Buttons/RX/{TOPICS_BUTTON[i]['name']}",
                10,
            ).publish(button_value)

    def send_msg(self):
        raise NotImplementedError

    def on_message_received(self, var1):
        """
        callback function that is called when message is received
        """

        # print(var1)

        self.get_logger().debug(var1)

        return

        # stop if xbee is disabed
        if self.__disabled:
            return

        # get message from the physical xbee
        # message = None
        # try:
        #     message = self.__xbee_device.read_data(0.0004)
        # except TimeoutException:
        #     return
        # except Exception as e:
        #     print("\n\nBIG ISSUE\n")
        #     print(e)
        #     return

        # stop if message is none
        if xbee_message is None:
            return

        # if start of message is not valid, stop
        if list(xbee_message.data)[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
            return

        if not self.__is_first_connected:
            self.__is_first_connected = True

        self.__parse_incoming_message(list(xbee_message.data)[1:])

        self.__last_successful_message = time.time_ns()

    def run(self):
        # last_cycle_time = time.time_ns()
        self.__xbee_device.add_data_received_callback(self.on_message_received)
        rclpy.spin(self)

        # while not self.__disabled:
        #     if time.time_ns() - last_cycle_time > XBEE_UPDATE_RATE:
        #         last_cycle_time = time.time_ns()

        #         if (
        #             time.time_ns() - self.__last_successful_message > XBEE_TIMEOUT
        #             and self.__is_first_connected
        #         ):
        #             self.set_disabled(True)

        rclpy.shutdown()


def main():
    rclpy.init()
    xbee = Xbee()
    xbee.run()


if __name__ == "__main__":
    main()


# import rclpy
# import serial
# from rclpy import Node
# from std_msgs.msg import String
# from constants.RoverConstants import PORT, BAUD_RATE


# class XBee(Node):
#     #__xbee: serial.Serial

#     def __init__(self):
#         super().__init__("XBee")
#         # self.__xbee = serial.Serial(PORT, BAUD_RATE)

#         # self.create_subscription(String, "/sensors/GPS/TX", self.send_gps, 10)

#         self.run()

#     def send_gps(self, data: String):
#         print(data)

#     def run(self):
#         rclpy.spin(self)


# def main():
#     xbee = XBee()


# if __name__ == "__main__":
#     main()
