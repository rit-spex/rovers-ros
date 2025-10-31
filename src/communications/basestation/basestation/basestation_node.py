#!/usr/bin/env python3

import time

# ros imports
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.publisher import Publisher
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, Float32, Int16MultiArray

from basestation_communication import BaseStationCommunications

from constants.CommandCodes import TOPICS_JOYSTICK, TOPICS_BUTTON, CONSTANTS
from constants.CAN_Constants import CHANNEL, TOPICS

import numpy as np

XBEE_TIMEOUT = 1000000000  # 1,000,000 nano second -> 1 second



class Basestation(Node):
    __basestation_communications: BaseStationCommunications

    __publishers: list[list[rclpy.publisher.Publisher]]

    __subscription: rclpy.subscription.Subscription

    def __init__(self):
        super().__init__("Basestation")

        self.__basestation_communications = BaseStationCommunications()

        self.__publishers = [
            [
                self.create_publisher(self.__basestation_communications.python_to_interface(value_type), f"/BASESTATION/{message.name}/{value_name}", 10)
                for (value_name, value_type) in message.values
            ]
            for message in self.__basestation_communications.get_message()
        ]

        self.__subscription = self.create_subscription(
            msg_type=Int16MultiArray,
            topic="/XBEE/MESSAGES",
            callback=self.on_message_received,
            qos_profile=10,
        )

    def __on_message_received(self, message: Int16MultiArray):
        """
        callback function that is called when message is received
        """

        # if start of message is not valid, stop
        if list(message.data)[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
            return

        # split the message data into a list
        data = list(message.data)

        self.get_logger().info(str(data))

        # check if message has a valid start message
        if data[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
            self.get_logger().info(f"not valid start message")
            return
        elif data[0] == int.from_bytes(CONSTANTS.QUIT_MESSAGE, "big"):
            return

        self.__parse_incoming_message(list(message.data)[1:])
        self.get_logger().info("receive:")
        for i, byte in enumerate(data):
            self.get_logger().info(f"{i}, {bin(byte)}")
        self.get_logger().info("")

        self.__parse_incoming_message(data)

        # flag to make so it won't error out immeadately
        if not self.__is_first_connected:
            self.__is_first_connected = True

        self.__last_successful_message = time.time_ns()





class Basestation(Node):
    # flag to determine if the xbee should be disabled based on no signal
    __disabled: bool

    # flag to be triggered once the xbee has received any data
    __is_first_connected: bool

    # track when the last successful message was received
    __last_successful_message: int

    # all the current values from the xbee
    # self.__button_values = [False] * CONSTANTS.NUM_BUTTONS
    __axis_values: list[float]

    # this will call be an interrupt to process the message
    __subscription: rclpy.subscription.Subscription


    def __init__(self):
        super().__init__("Basestation_node")

        self.__disabled = False
        self.__is_first_connected = False

        # create the subscriber
        self.__subscription = self.create_subscription(
            msg_type=Int16MultiArray,
            topic="/XBEE/MESSAGES",
            callback=self.on_message_received,
            qos_profile=10,
        )

        # TODO: remove when updating basestation code
        self.__button_values = [False] * CONSTANTS.XBOX.NUM_BUTTONS
        self.__axis_values = [0.0] * CONSTANTS.XBOX.NUM_AXES
        self.__value_state = {"xbox": {}, "n64": {}}
        self.__name_to_ID_XBOX = {}

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

        # the current byte number
        byte_num: int = 0

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

            value = (message[byte_num] - 100.0) / (100.0)
            byte_num = byte_num + 1

            ros_msg = TOPICS_JOYSTICK[i]["val"]()
            ros_msg.data = value

            self.__publish_new_controls("xbox", TOPICS_JOYSTICK[i]["name"], ros_msg)
            self.__joystick_publishers[i].publish(ros_msg)

        # parse for button values
        for i in range(0, CONSTANTS.XBOX.NUM_BUTTONS, 1):
            if i != 0 and i % 4 == 0:
                byte_num = byte_num + 1

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

            self.__publish_new_controls("xbox", TOPICS_BUTTON[i]["name"], ros_msg)

        # parsing N64 controller
        # A B L R
        # CU CD CL CR
        # DU DD DL DR
        # Z

        a_value = Bool()
        a_value.data = bool((n64_message[0] << 6) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "A", a_value)
        b_value = Bool()
        b_value.data = bool((n64_message[0] << 4) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "B", b_value)
        l_value = Bool()
        l_value.data = bool((n64_message[0] << 2) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "L", l_value)
        r_value = Bool()
        r_value.data = bool((n64_message[0] << 0) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "R", r_value)

        cu_value = Bool()
        cu_value.data = bool((n64_message[1] << 6) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "CU", cu_value)
        cd_value = Bool()
        cd_value.data = bool((n64_message[1] << 4) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "CD", cd_value)
        cl_value = Bool()
        cl_value.data = bool((n64_message[1] << 2) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "CL", cl_value)
        cr_value = Bool()
        cr_value.data = bool((n64_message[1] << 0) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "CR", cr_value)

        du_value = Bool()
        du_value.data = bool((n64_message[2] << 6) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "DU", du_value)
        dd_value = Bool()
        dd_value.data = bool((n64_message[2] << 4) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "DD", dd_value)
        dl_value = Bool()
        dl_value.data = bool((n64_message[2] << 2) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "DL", dl_value)
        dr_value = Bool()
        dr_value.data = bool((n64_message[2] << 0) >> 6 == CONSTANTS.N64.BUTTONS.ON)
        self.__publish_new_controls("n64", "DR", dr_value)

        # z_value = Bool()
        # z_value.data = bool(n64_message[3] == CONSTANTS.N64.BUTTONS.ON)
        # self.__publish_new_controls("n64", "Z", z_value)

    def send_msg(self):
        raise NotImplementedError

    def on_message_received(self, message):
        """
        callback function that is called when message is received
        """

        # xbee is disabled
        # if self.__is_disabled:
        #     self.get_logger().info(f"xbee is disabled, returning...")
        #     return

        # message is invalid
        if message is None:
            self.get_logger().info(f"message was none")
            return

        # if start of message is not valid, stop
        if list(message.data)[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
            return

        # split the message data into a list
        data = list(message.data)

        self.get_logger().info(str(data))

        # check if message has a valid start message
        if data[0] != int.from_bytes(CONSTANTS.START_MESSAGE, "big"):
            self.get_logger().info(f"not valid start message")
            return
        elif data[0] == int.from_bytes(CONSTANTS.QUIT_MESSAGE, "big"):
            return

        self.__parse_incoming_message(list(message.data)[1:])
        self.get_logger().info("receive:")
        for i, byte in enumerate(data):
            self.get_logger().info(f"{i}, {bin(byte)}")
        self.get_logger().info("")

        self.__parse_incoming_message(data)

        # flag to make so it won't error out immeadately
        if not self.__is_first_connected:
            self.__is_first_connected = True

        self.__last_successful_message = time.time_ns()

    def run(self):
        self.get_logger().info("starting basestation ...")
        rclpy.spin(self)


def main():
    rclpy.init()
    basestation = Basestation()
    basestation.run()

if __name__ == "__main__":
    main()
