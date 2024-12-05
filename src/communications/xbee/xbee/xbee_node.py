#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import TimeoutException

import constants.CommanndCodes
from constants.CommanndCodes import CONSTANTS

import time

# ros imports
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# from custom_interfaces.msg import CanFD, Can
from constants.CommanndCodes import TOPICS_JOYSTICK, TOPICS_BUTTON
import rclpy.publisher
import rclpy.subscription

XBEE_PORT = "/dev/ttyUSB0"
XBEE_SPEED = 921600
XBEE_UPDATE_RATE = 40000000  # 40000 nano second -> 40 micro second
XBEE_TIMEOUT = 1000000000  # 1,000,000 nano second -> 1 second


class Xbee(Node):
    def __init__(self):

        super().__init__("Xbee_node")

        # the number of iterations without signal
        self.__num_no_signal = 0

        # flag to determine if the xbee should be disabled based
        # on no signal
        self.__is_disabled = False

        # flag to be triggered once the xbee has received any data
        self.__is_first_connected = False

        # all the current values from the xbee
        # self.__button_values = [False] * CONSTANTS.NUM_BUTTONS
        self.__axis_values = [0.0] * CONSTANTS.NUM_AXES

        # open the port to the device
        self.__xbee_device = XBeeDevice(XBEE_PORT, XBEE_SPEED)
        self.__xbee_device.open()

        # track when the last successful message was received
        self.__last_successful_message = time.time_ns()


    def __del__(self):
        """
        have the device port be closed
        """
        self.__xbee_device.close()

    # """
    # get the current value of selected input

    # :param input_type - Specifies what type of input
    # :param input_trigger - Controller button or axis, value from CommandCodes
    # """

    # def get_current_value(self, input_type: int, input_trigger: int) -> float | bool:
    #     match input_type:
    #         case CONSTANTS.INPUT_TYPE.IS_AXIS:
    #             if input_trigger == CONSTANTS.JOYSTICK.AXIS_LY:
    #                 return self.__axis_values[0]
    #             else:
    #                 return self.__axis_values[1]
    #         case CONSTANTS.INPUT_TYPE.IS_AXIS:
    #             return self.__button_values[input_trigger]
    #         case CONSTANTS.INPUT_TYPE.IS_BUTTON:
    #             return self.__button_values[
    #                 CONSTANTS.NUM_BUTTONS
    #                 - CONSTANTS.NUM_TRIGGER
    #                 + (input_trigger - CONSTANTS.NUM_AXES)
    #             ]

    #     return False

    # print all the current values to terminal
    # def print_values(self) -> None:

    #     print(self.__button_values)

    #     print(
    #         "Left Axis: ",
    #         self.get_current_value(
    #             CONSTANTS.INPUT_TYPE.IS_AXIS, CONSTANTS.JOYSTICK.AXIS_LY
    #         ),
    #     )

    #     print(
    #         " Right Axis: ",
    #         self.get_current_value(
    #             CONSTANTS.INPUT_TYPE.IS_AXIS, CONSTANTS.JOYSTICK.AXIS_RY
    #         ),
    #     )

    #     print(
    #         " A Button: ",
    #         self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.A),
    #     )

    #     print(
    #         " B Button: ",
    #         self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.B),
    #     )

    #     print(
    #         " X Button: ",
    #         self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.X),
    #     )

    #     print(
    #         " Y Button: ",
    #         self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.Y),
    #     )

    #     print(
    #         " LB Button: ",
    #         self.get_current_value(
    #             CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.LEFT_BUMPER
    #         ),
    #     )

    #     print(
    #         " RB Button: ",
    #         self.get_current_value(
    #             CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.RIGHT_BUMPER
    #         ),
    #     )

    #     print(
    #         " LT Button: ",
    #         self.get_current_value(
    #             CONSTANTS.INPUT_TYPE.IS_TRIGGER, CONSTANTS.TRIGGER.AXIS_LT
    #         ),
    #     )

    #     print(
    #         " RT Button: ",
    #         self.get_current_value(
    #             CONSTANTS.INPUT_TYPE.IS_TRIGGER, CONSTANTS.TRIGGER.AXIS_RT
    #         ),
    #     )

    #     print()

    # checks if the xbee is disabled
    def is_disabled(self) -> bool:
        return self.__is_disabled

    # clears the disable flag to allow the xbee to continue normal function
    def clear_disable(self) -> None:
        self.__is_disabled = False

    # disable the xbee
    def disable_xbee(self) -> None:
        self.__is_disabled = True


    #
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
                10
            ).publish(button_value)

    def send_msg(self):
        pass

    def on_message_received(self):
        """
        callback function that is called when message is received
        """
        # xbee is disabled
        if self.__is_disabled:
            return

        message = None

        # get message from the physical xbee
        try:
            message = self.__xbee_device.read_data(0.0004)
        except TimeoutException:
            return
        except Exception as e:
            print("\n\nBIG ISSUE\n")
            print(e)
            return

        # message is invalid
        if message is None:
            return

        # check if message has a valid start message
        if list(message.data)[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
            return
        else:
            if not self.__is_first_connected:
                self.__is_first_connected = True

            self.__parse_incoming_message(list(message.data)[1:])
            # self.print_values()

            self.__last_successful_message = time.time_ns()

            # message_id = 3

            # ros_msg = Can()
            # ros_msg.channel = TOPICS[message_id]["channel"]
            # ros_msg.id = message_id
            # ros_msg.buf = [
            #     int(
            #         (
            #             self.get_current_value(
            #                 CONSTANTS.JOYSTICK.AXIS_LY, CONSTANTS.INPUT_TYPE.IS_AXIS
            #             )
            #             * 100
            #         )
            #         + 100
            #     ),
            #     int(
            #         (
            #             self.get_current_value(
            #                 CONSTANTS.JOYSTICK.AXIS_RY, CONSTANTS.INPUT_TYPE.IS_AXIS
            #             )
            #             * 100
            #         )
            #         + 100
            #     ),
            #     0,
            #     0,
            #     0,
            #     0,
            #     0,
            #     0,
            # ]

            # self.create_publisher(
            #     msg_type=Can,
            #     topic=f"/CAN/TX/{TOPICS[message_id]['name']}",
            #     qos_profile=10,
            # ).publish(ros_msg)

    def run(self):
        # rclpy.spin(self)
        last_cycle_time = time.time_ns()

        while not self.__is_disabled:
            if time.time_ns() - last_cycle_time > XBEE_UPDATE_RATE:
                last_cycle_time = time.time_ns()
                self.on_message_received()

                if (
                    time.time_ns() - self.__last_successful_message > XBEE_TIMEOUT
                    and self.__is_first_connected
                ):
                    self.disable_xbee()


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
