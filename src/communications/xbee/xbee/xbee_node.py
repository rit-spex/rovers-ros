#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import TimeoutException

from constants.CAN_Constants import TOPICS
from constants.CommanndCodes import CONSTANTS

import time

# ros imports
from pyparsing import OnlyOnce
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import CanFD, Can
from constants.CommanndCodes import TOPICS_JOYSTICK, TOPICS_BUTTON
import rclpy.publisher
import rclpy.subscription

XBEE_PORT = "/dev/ttyUSB0"
XBEE_SPEED = 230400
XBEE_UPDATE_RATE = 40000000  # 40000 nano second -> 40 micro second
XBEE_TIMEOUT = 1000000000  # 1,000,000 nano second -> 1 second


class Xbee(Node):
    def __init__(self):

        super().__init__("xbee_node")

        # the number of iterations without signal
        self.__num_no_signal = 0

        # flag to determine if the xbee should be disabled based
        # on no signal
        self.__is_disabled = False

        # flag to be triggered once the xbee has received any data
        self.__is_first_connected = False

        # all the current values from the xbee
        self.__button_values = [False] * CONSTANTS.NUM_BUTTONS
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

    def get_current_value(self, input_type: int, input_trigger: int) -> float | bool:
        match input_type:
            case CONSTANTS.INPUT_TYPE.IS_AXIS:
                if input_trigger == CONSTANTS.JOYSTICK.AXIS_LY:
                    return self.__axis_values[0]
                else:
                    return self.__axis_values[1]
            case CONSTANTS.INPUT_TYPE.IS_TRIGGER:
                return self.__button_values[input_trigger]
            case CONSTANTS.INPUT_TYPE.IS_BUTTON:
                return self.__button_values[
                    CONSTANTS.NUM_BUTTONS
                    - CONSTANTS.NUM_TRIGGER
                    + (input_trigger - CONSTANTS.NUM_AXES)
                ]

        return False

    def print_values(self) -> None:
        """
        print all the current values to terminal
        """
        # self.get_logger().info(self.__button_values)

        self.get_logger().info(
            f"Left Axis: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_AXIS, CONSTANTS.JOYSTICK.AXIS_LY)}"
        )

        self.get_logger().info(
            f"Right Axis: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_AXIS, CONSTANTS.JOYSTICK.AXIS_RY)}"
        )

        self.get_logger().info(
            f"A Button: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.A)}"
        )

        self.get_logger().info(
            f"B Button: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.B)}"
        )

        self.get_logger().info(
            f"X Button: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.X)}"
        )

        self.get_logger().info(
            f"Y Button: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.Y)}"
        )

        self.get_logger().info(
            f"LB Button: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.LEFT_BUMPER)}"
        )

        self.get_logger().info(
            f"RB Button: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_BUTTON, CONSTANTS.BUTTONS.RIGHT_BUMPER)}"
        )

        self.get_logger().info(
            f"LT Button: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_TRIGGER, CONSTANTS.TRIGGER.AXIS_LT)}"
        )

        self.get_logger().info(
            f"RT Button: {self.get_current_value(CONSTANTS.INPUT_TYPE.IS_TRIGGER, CONSTANTS.TRIGGER.AXIS_RT)}\n"
        )

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

                # self.get_logger().info(f"i: {i+1}")
                value = (message[byte_num] - 100.0) / (100.0)
                self.__axis_values[i] = value
                byte_num = byte_num + 1

                ros_msg = TOPICS_JOYSTICK[i+1]["val"]()
                ros_msg.data = value

                # self.get_logger().info(f"0: {TOPICS_JOYSTICK[0]}")
                # self.get_logger().info(f"1: {TOPICS_JOYSTICK[1]}")
                # self.get_logger().info(f"2: {TOPICS_JOYSTICK[2]}")
                # self.get_logger().info(f"3: {TOPICS_JOYSTICK[3]}")

                # self.get_logger().info(f"{TOPICS_JOYSTICK[i+1]['name']}: {value}")

                self.create_publisher(
                    TOPICS_JOYSTICK[i+1]["val"],
                    f"/Xbee/RX/Controller/Axis/{TOPICS_JOYSTICK[i+1]['name']}",
                    10,
                ).publish(ros_msg)

        # parse for button values
        for i in range(0, CONSTANTS.NUM_BUTTONS, 1):
            if i != 0 and i % 4 == 0:
                byte_num = byte_num + 1

            # check if section of byte is on or off
            # self.get_logger().info(str(message[byte_num]))

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

            ros_msg = TOPICS_BUTTON[i]["val"]()
            ros_msg.data = button_value

            # self.get_logger().info(f"{TOPICS_BUTTON[i]['name']}: {button_value}")

            self.create_publisher(
                TOPICS_BUTTON[i]["val"],
                f"/Xbee/RX/Controller/Buttons/{TOPICS_BUTTON[i]['name']}",
                10,
            ).publish(ros_msg)

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
            self.get_logger().info("\n\nBIG ISSUE\n")
            self.get_logger().info(str(e))
            return

        # message is invalid
        if message is None:
            return

        # check if message has a valid start message
        if list(message.data)[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
            return

        if not self.__is_first_connected:
            self.__is_first_connected = True

        self.get_logger().info(f"parsing {list(message.data)[1:]}")
        self.__parse_incoming_message(list(message.data)[1:])
        # self.print_values()

        self.__last_successful_message = time.time_ns()

        # message_id = 3

    def run(self):
        last_cycle_time = time.time_ns()

        self.get_logger().info("starting xbee...")

        while not self.__is_disabled:
            if time.time_ns() - last_cycle_time > XBEE_UPDATE_RATE:
                last_cycle_time = time.time_ns()
                self.on_message_received()

                if (
                    time.time_ns() - self.__last_successful_message > XBEE_TIMEOUT
                    and self.__is_first_connected
                ):
                    self.get_logger().info("disabling xbee")
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
#         self.get_logger().info(data)

#     def run(self):
#         rclpy.spin(self)


# def main():
#     xbee = XBee()


# if __name__ == "__main__":
#     main()
