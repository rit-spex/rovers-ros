#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import TimeoutException

from constants.CAN_Constants import CHANNEL, TOPICS
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
from std_msgs.msg import Bool

import numpy as np

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
        self.__button_values = [False] * CONSTANTS.XBOX.NUM_BUTTONS
        self.__axis_values = [0.0] * CONSTANTS.XBOX.NUM_AXES

        # open the port to the device
        self.__xbee_device = XBeeDevice(XBEE_PORT, XBEE_SPEED)
        self.__xbee_device.open()

        # track when the last successful message was received
        self.__last_successful_message = time.time_ns()

        self.__value_state = {
            "xbox": {},
            "n64": {}
        }

        self.__name_to_ID_XBOX = {}

        # creates publishers for all the different buttons
        self.__joystick_publishers: dict[int, Publisher] = {}
        for joystick in CONSTANTS.XBOX.JOYSTICK.LIST_OF_AXIS:
            self.__joystick_publishers[joystick] = self.create_publisher(
                TOPICS_JOYSTICK[joystick]["val"],
                f"/Xbee/RX/Xbox/Axis/{TOPICS_JOYSTICK[joystick]['name']}",
                10,
            )
            self.__name_to_ID_XBOX[TOPICS_JOYSTICK[joystick]['name']] = joystick
            self.__value_state["xbox"][TOPICS_JOYSTICK[joystick]['name']] = CONSTANTS.XBOX.JOYSTICK.NEUTRAL_HEX
        # self.__trigger_publishers: dict[int, Publisher] = {}
        # for trigger in CONSTANTS.TRIGGER.LIST_OF_TRIGGERS:
        #     self.__trigger_publishers[trigger] = self.create_publisher(
        #         TOPICS_BUTTON[trigger]["val"],
        #         f"/Xbee/Xbox/{TOPICS_TRIGGER[trigger]['name']}",
        #         10,
        #     )
        self.__button_publishers: dict[int, Publisher] = {}
        for button in CONSTANTS.XBOX.BUTTONS.LIST_OF_BUTTONS:
            self.__button_publishers[button] = self.create_publisher(
                TOPICS_BUTTON[button]["val"],
                f"/Xbee/RX/Xbox/Buttons/{TOPICS_BUTTON[button]['name']}",
                10,
            )
            self.__name_to_ID_XBOX[TOPICS_BUTTON[button]['name']] = button
            self.__value_state['xbox'][TOPICS_BUTTON[button]['name']] = 0

        self.__n64_publishers = {
            "A": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/A", 10),
            "N": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/B", 10),
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

            "Z": self.create_publisher(Bool, "/Xbee/RX/N64/Buttons/Z", 10)
        }

        self.__value_state['n64'] = {
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
                if input_trigger == CONSTANTS.XBOX.JOYSTICK.AXIS_LY:
                    return self.__axis_values[0]
                else:
                    return self.__axis_values[1]
            case CONSTANTS.INPUT_TYPE.IS_TRIGGER:
                return self.__button_values[input_trigger]
            case CONSTANTS.INPUT_TYPE.IS_BUTTON:
                return self.__button_values[
                    CONSTANTS.XBOX.NUM_BUTTONS
                    - CONSTANTS.XBOX.NUM_TRIGGER
                    + (input_trigger - CONSTANTS.XBOX.NUM_AXES)
                ]

        return False

    # checks if the xbee is disabled
    def is_disabled(self) -> bool:
        return self.__is_disabled

    # clears the disable flag to allow the xbee to continue normal function
    def clear_disable(self) -> None:
        self.__is_disabled = False

    # disable the xbee
    def disable_xbee(self) -> None:
        self.__is_disabled = True

    def __publish_new_controls(self, controller: str, button: str, value) -> None:
        """Helper function to only publish updated information

        Args:
            controller (str): the name of the controller this value is for ('xbox', 'n64')
            button (str): the name of the button this value this for
            value (Any): the value to publish
        """

        if self.__value_state[controller][button] == value:
            return

        if controller == "n64":
            self.__n64_publishers[button].publish(value)

        if "axis" in button.lower():
            self.__joystick_publishers[self.__name_to_ID_XBOX[button]].publish(value)
        self.__button_publishers[self.__name_to_ID_XBOX[button]].publish(value)

    def __parse_incoming_message(self, message: list[int]):
        """
        Helper function that is called when message is received, to parse to get values

        Args:
            message (list[int]): a full message that start with start message
        """

        # the current byte number

        byte_num: int = 0

        # self.get_logger().info("parse:")
        # for i, byte in enumerate(message):
        #     self.get_logger().info(f"{i}, {bin(byte)}")
        # self.get_logger().info("")

        n64_message: list[np.uint8] = [np.uint8(m) for m in message[6:]]
        message = message[1:5]

        # parse for axis
        for i in CONSTANTS.XBOX.JOYSTICK.LIST_OF_AXIS:
            if not (
                CONSTANTS.XBOX.JOYSTICK.MAX_VALUE
                >= message[byte_num]
                >= CONSTANTS.XBOX.JOYSTICK.MIN_VALUE
            ):
                continue
            # self.get_logger().info(f"i: {i+1}")
            value = (message[byte_num] - 100.0) / (100.0)
            # self.__axis_values[i] = value
            byte_num = byte_num + 1

            ros_msg = TOPICS_JOYSTICK[i]["val"]()
            ros_msg.data = value

            # self.get_logger().info(f"publishing /Xbee/RX/Xbox/Axis/{TOPICS_JOYSTICK[i]['name']}: {value}")

            self.__publish_new_controls('xbox',TOPICS_JOYSTICK[i]['name'] , ros_msg)
            self.__joystick_publishers[i].publish(ros_msg)

        # parse for button values

        # self.get_logger().info(f"first set: {bin(message[2])}")
        # self.get_logger().info(f"secon set: {bin(message[3])}")
        for i in range(0, CONSTANTS.XBOX.NUM_BUTTONS, 1):
            if i != 0 and i % 4 == 0:
                byte_num = byte_num + 1

            # check if section of byte is on or off
            # self.get_logger().info(f"byte num: {byte_num}")
            # self.get_logger().info(f"message: {message}")
            # self.get_logger().info(f"together: {message[byte_num]}")

            button_value = (
                (
                    message[byte_num]
                    // pow(
                        2,
                        (i % CONSTANTS.XBOX.BUTTONS.NUM_BUTTONS_PER_BYTE)
                        * CONSTANTS.XBOX.BUTTONS.SIZE_BUTTON_IN_BITS,
                    )
                )
                % 4
            ) == CONSTANTS.XBOX.BUTTONS.ON

            ros_msg = TOPICS_BUTTON[i]["val"]()
            ros_msg.data = button_value

            # self.get_logger().info(f"publishing /Xbee/RX/Xbox/Buttons/{TOPICS_BUTTON[i]['name']}: {button_value}")

            # self.create_publisher(
            #     TOPICS_BUTTON[i]["val"],
            #     f"/Xbee/RX/Controller/Buttons/{TOPICS_BUTTON[i]['name']}",
            #     10,
            # ).publish(ros_msg)

            # self.__button_publishers[i].publish(ros_msg)
            self.__publish_new_controls('xbox',TOPICS_BUTTON[i]['name'] , ros_msg)

        # parsing N64 controller
        # A B L R
        # CU CD CL CR
        # DU DD DL DR
        # Z

        a_value = Bool()
        a_value.data = bool((n64_message[0] << 6) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "a", a_value)
        b_value = Bool()
        b_value.data = bool((n64_message[0] << 4) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "b", b_value)
        l_value = Bool()
        l_value.data = bool((n64_message[0] << 2) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "l", l_value)
        r_value = Bool()
        r_value.data = bool((n64_message[0] << 0) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "r", r_value)

        cu_value = Bool()
        cu_value.data = bool((n64_message[1] << 6) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "cu", cu_value)
        cd_value = Bool()
        cd_value.data = bool((n64_message[1] << 4) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "cd", cd_value)
        cl_value = Bool()
        cl_value.data = bool((n64_message[1] << 2) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "cl", cl_value)
        cr_value = Bool()
        cr_value.data = bool((n64_message[1] << 0) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "cr", cr_value)

        du_value = Bool()
        du_value.data = bool((n64_message[2] << 6) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "du", du_value)
        dd_value = Bool()
        dd_value.data = bool((n64_message[2] << 4) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "dd", dd_value)
        dl_value = Bool()
        dl_value.data = bool((n64_message[2] << 2) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "dl", dl_value)
        dr_value = Bool()
        dr_value.data = bool((n64_message[2] << 0) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "dr", dr_value)

        z_value = Bool()
        z_value.data = bool(n64_message[3] == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls('n64', "z", z_value)

    def send_msg(self):
        pass

    def on_message_received(self):
        """
        callback function that is called when message is received
        """
        # self.get_logger().info("starting message received")

        # xbee is disabled
        if self.__is_disabled:
            self.get_logger().info(f"xbee is disabled, returning...")
            return

        message = None

        # get message from the physical xbee
        try:
            message = self.__xbee_device.read_data(0.0004)
        except TimeoutException:
            # self.get_logger().info("timed out")
            return
        except Exception as e:
            self.get_logger().info("\n\nBIG ISSUE\n")
            self.get_logger().info(str(e))
            return

        # message is invalid
        if message is None:
            self.get_logger().info(f"message was none")
            return

        # split the message data into a list
        data = list(message.data)
        # self.get_logger().info(f"got data: {data}")

        # check if message has a valid start message
        if data[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
            self.get_logger().info(f"not valid start message")
            return
        elif data[0] == int.from_bytes(CONSTANTS.QUIT_MESSAGE, "big"):
            self.disable_xbee()
            return

        if not self.__is_first_connected:
            self.__is_first_connected = True

        self.get_logger().info("receive:")
        for i, byte in enumerate(data):
            self.get_logger().info(f"{i}, {bin(byte)}")
        self.get_logger().info("")

        self.__parse_incoming_message(data)
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

                # if (
                #     time.time_ns() - self.__last_successful_message > XBEE_TIMEOUT
                #     and self.__is_first_connected
                # ):
                #     self.get_logger().info("disabling xbee")
                #     self.disable_xbee()

        # Signalling E-STOP
        ros_msg = Can()
        ros_msg.id = 0
        ros_msg.channel = CHANNEL.MAIN_BODY
        ros_msg.buf = [0, 0, 0, 0, 0, 0, 0, 0]
        self.create_publisher(Can, "/CAN/TX/E_STOP", 10).publish(ros_msg)


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
