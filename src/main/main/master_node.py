#!/usr/bin/env python3

from numpy import int32

# ros imports
import rclpy
import rclpy.logging
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, UInt8, UInt16
from custom_interfaces.msg import Can

from constants.CommandCodes import CONSTANTS
from constants.can_encoding import TeensyCommunication
from constants.CAN_constants import CAN_MESSAGE_IDS, TEENSY_CAN_MESSAGES, ArmState, ArmDirection, Subsystems_Names, SubsystemsIDs
from constants.CAN_structs import Signal, Message

timeout_duration = 1  # seconds


class Master(Node):

    __estop_publisher: rclpy.publisher.Publisher
    __chassis_node_enable_publisher: rclpy.publisher.Publisher # This is sent to other nodes to tell them to enable/disable the chassis
    __chassis_teensy_enable_publisher: rclpy.publisher.Publisher
    __arm_node_enable_publisher: rclpy.publisher.Publisher # This is sent to other nodes to tell them to enable/disable the arm
    __arm_teensy_enable_publisher: rclpy.publisher.Publisher
    __science_node_enable_publisher: rclpy.publisher.Publisher # This is sent to other nodes to tell them to enable/disable the science subsystem
    __science_teensy_enable_publisher: rclpy.publisher.Publisher
    __auto_state_publisher: rclpy.publisher.Publisher

    __ros_teensy_publisher: rclpy.publisher.Publisher # This is sent to the teensy to tell it that the ROS system is alive

    # basestation heartbeat tracking
    __last_basestation_heartbeat_time: UInt16
    __current_basestation_heartbeat_time: UInt16
    __basestation_recieved_heartbeat: bool

    # heartbeat tracking
    __last_heartbeat_time: int
    __current_heartbeat_time: int
    __received_heartbeat: bool
    # subsystem status tracking
    __chassis_teensy_enabled: bool # we wait until the chassis teensy sends a heartbeat message before we consider the chassis enabled
    __chassis_node_enabled: bool   # wait until teensy knows it is enabled before we consider the chassis node enabled
    __last_chassis_command: UInt16
    __curr_chassis_command: UInt16

    __arm_teensy_enabled: bool     # we wait until the arm teensy sends a heartbeat message before we consider the arm enabled
    __arm_node_enabled: bool       # wait until teensy knows it is enabled before we consider the arm node enabled
    __last_arm_command: UInt16
    __curr_arm_command: UInt16

    __science_teensy_enabled: bool # we wait until the science teensy sends a heartbeat message before we consider the science subsystem enabled
    __science_node_enabled: bool   # wait until teensy knows it is enabled before we consider the science node enabled
    __last_science_command: UInt16
    __curr_science_command: UInt16

    def __init__(self):
        super().__init__("Master")
        self.__last_basestation_heartbeat_time = UInt16()
        self.__current_basestation_heartbeat_time = UInt16()
        self.__basestation_recieved_heartbeat = False

        self.__last_heartbeat_time = 0
        self.__current_heartbeat_time = 0
        self.__received_heartbeat = False
        self.__chassis_teensy_enabled = False
        self.__chassis_node_enabled = False
        self.__last_chassis_command = UInt16()
        self.__curr_chassis_command = UInt16()

        self.__arm_teensy_enabled = False
        self.__arm_node_enabled = False
        self.__last_arm_command = UInt16()
        self.__curr_arm_command = UInt16()

        self.__science_teensy_enabled = False
        self.__science_node_enabled = False
        self.__last_science_command = UInt16()
        self.__curr_science_command = UInt16()

        # create publishers and subscriptions for e-stop, chassis, arm, and science commands
        self.__estop_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/ESTOP",
            qos_profile=10,
        )
        self.__auto_state_publisher = self.create_publisher(
            msg_type=UInt8,
            topic="/ROVER/AUTO_STATE",
            qos_profile=10,
        )
        self.create_subscription(

        self.__chassis_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/BASESTATION/" + CONSTANTS.QUIT.NAME + "/" + CONSTANTS.QUIT.QUIT_MESSAGE,
            topic="/CHASSIS/ENABLED",
            qos_profile=10,
        )

        self.__chassis_teensy_enable_publisher = self.create_publisher(
            msg_type=Can,
            topic= TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_CHASSIS].topic_name,
            qos_profile=10,
        )

        self.__arm_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/ARM/ENABLED",
            qos_profile=10,
        )

        self.__arm_teensy_enable_publisher = self.create_publisher(
            msg_type=Can,
            topic= TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_ARM].topic_name,
            qos_profile=10,
        )

        self.__science_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/SCIENCE/ENABLED",
            qos_profile=10,
        )

        self.__science_teensy_enable_publisher = self.create_publisher(
            msg_type=Can,
            topic= TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_SCIENCE].topic_name,
            qos_profile=10,
        )

        self.__ros_teensy_publisher = self.create_publisher(
            msg_type=Can,
            topic= TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ROS_HEARTBEAT].topic_name,
            qos_profile=10,
        )

        # create subscriptions for basestation heartbeat and quit messages
        self.create_subscription(
            msg_type=Bool,
            topic="/BASESTATION/" + CONSTANTS.QUIT.NAME + "/" + CONSTANTS.QUIT.NAME,
            callback=self.__on_quit_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=UInt16,
            topic="/BASESTATION/"
            + CONSTANTS.HEARTBEAT.NAME
            + "/"
            + CONSTANTS.HEARTBEAT.TIMESTAMP_MESSAGE,
            callback=self.__on_basestation_heartbeat_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=UInt8,
            topic="/BASESTATION/auto_state/auto_state",
            callback=self.__on_auto_state_received,
            qos_profile=10,
        )
        self.create_subscription(
        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )
        self.create_timer(
            timer_period_sec=timeout_duration,
            callback=self.__check_timeout,
        )

        # create subscriptions for subsystem commands
        for (message_id, message) in TEENSY_CAN_MESSAGES.items():
            if(message.subsystem == Subsystems_Names.GENERAL and message.isforJetson):
                self.create_subscription(
                    msg_type=Can,
                    topic=message.topic_name,
                    callback=self.__on_new_CAN_message_received,
                    qos_profile=10
                )

    def __check_timeout(self) -> None:
        # check for basestation heartbeat timeout
        if self.__basestation_recieved_heartbeat:
            if self.__current_basestation_heartbeat_time == self.__last_basestation_heartbeat_time:
                self.get_logger().info("Heartbeat timeout detected")
                estop_msg = Bool()
                estop_msg.data = True
                self.__estop_publisher.publish(estop_msg)
            else:
                self.__last_basestation_heartbeat_time = self.__current_basestation_heartbeat_time

        # check for chassis command timeout
        if self.__chassis_teensy_enabled:
            if self.__curr_chassis_command == self.__last_chassis_command:
                self.get_logger().info("Chassis command timeout detected, disabling chassis")
                self.__chassis_node_enabled = False
                self.__chassis_teensy_enabled = False
                self.__chassis_node_enable_publisher.publish(Bool(data=False))
                self.__chassis_teensy_enable_publisher.publish(self.__pack_enable_message_to_teensy(CAN_MESSAGE_IDS.ENABLE_CHASSIS, False))
            else:
                self.__last_chassis_command = self.__curr_chassis_command
        
        # check for arm command timeout
        if self.__arm_teensy_enabled:
            if self.__curr_arm_command == self.__last_arm_command:
                self.get_logger().info("Arm command timeout detected, disabling arm")
                self.__arm_node_enabled = False
                self.__arm_teensy_enabled = False
                self.__arm_node_enable_publisher.publish(Bool(data=False))
                self.__arm_teensy_enable_publisher.publish(self.__pack_enable_message_to_teensy(CAN_MESSAGE_IDS.ENABLE_ARM, False))
            else:
                self.__last_arm_command = self.__curr_arm_command

        # check for science command timeout
        if self.__science_teensy_enabled:
            if self.__curr_science_command == self.__last_science_command:
                self.get_logger().info("Science command timeout detected, disabling science")
                self.__science_node_enabled = False
                self.__science_teensy_enabled = False
                self.__science_node_enable_publisher.publish(Bool(data=False))
                self.__science_teensy_enable_publisher.publish(self.__pack_enable_message_to_teensy(CAN_MESSAGE_IDS.ENABLE_SCIENCE, False))
            else:
                self.__last_science_command = self.__curr_science_command

        # send heartbeat message to teensy
        ros_heartbeat = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ROS_HEARTBEAT]
        ros_heartbeat.signals["timestamp"].set_value(int32((self.get_clock().now().nanoseconds // 1_000_000) % 100000000)) # set timestamp to current time in milliseconds

        can_packet = TeensyCommunication.encode_can_message(ros_heartbeat)
        if can_packet is not None:
            self.__ros_teensy_publisher.publish(can_packet)

    def __on_quit_received(self, msg: Bool):
        self.get_logger().info("Quit message received, forwarding e-stop ...")
        self.__estop_publisher.publish(msg)

    def __on_basestation_heartbeat_received(self, msg: UInt16):
        # self.get_logger().info("Heartbeat message received")
        self.__basestation_recieved_heartbeat = True
        self.__current_basestation_heartbeat_time = msg

    def __on_new_CAN_message_received(self, msg: Can):
        # first try to decode the message into a CAN message object
        can_message: Message | None
        try:
            can_message = TeensyCommunication.decode_can_packet(msg)
            if can_message is None:
                return
        except Exception as e:
            self.get_logger().error(f"Failed to decode CAN message: {e}")
            return

        # if e-stop message, forward it to the e-stop topic
        if(can_message.id == CAN_MESSAGE_IDS.E_STOP):
            self.get_logger().info("E-STOP message received from can_comms, forwarding ...")
            estop_msg = Bool()
            estop_msg.data = True
            self.__estop_publisher.publish(estop_msg)
        elif(can_message.id == CAN_MESSAGE_IDS.TEENSY_HEARTBEAT):
            try:
                source_signal = can_message.signals["source"]
                # self.get_logger().info(f"Source signal received: {can_message.toString()}")
                if(source_signal.value == SubsystemsIDs.CHASSIS):
                    self.__last_chassis_command = self.__curr_chassis_command
                    self.__curr_chassis_command = can_message.signals["timestamp"].value

                    # only set chassis enabled if it is not already
                    if(not self.__chassis_teensy_enabled):
                        self.__chassis_teensy_enabled = True
                        self.__chassis_teensy_enable_publisher.publish(self.__pack_enable_message_to_teensy(CAN_MESSAGE_IDS.ENABLE_CHASSIS, True))
                        self.get_logger().info("Chassis command received, enabling chassis teensy")
                    elif(can_message.signals["enabled"].value == 1 and not self.__chassis_node_enabled):
                        self.__chassis_node_enabled = True
                        self.__chassis_node_enable_publisher.publish(Bool(data=True))
                        self.get_logger().info("Chassis enabled message received from teensy, enabling chassis node")

                elif(source_signal.value == SubsystemsIDs.ARM):
                    self.__last_arm_command = self.__curr_arm_command
                    self.__curr_arm_command = can_message.signals["timestamp"].value

                    # only set arm enabled if it is not already
                    if(not self.__arm_teensy_enabled):
                        self.__arm_teensy_enabled = True
                        self.__arm_teensy_enable_publisher.publish(self.__pack_enable_message_to_teensy(CAN_MESSAGE_IDS.ENABLE_ARM, True))
                        self.get_logger().info("Arm command received, enabling arm teensy")
                    elif(can_message.signals["enabled"].value == 1 and not self.__arm_node_enabled):
                        self.__arm_node_enabled = True
                        self.__arm_node_enable_publisher.publish(Bool(data=True))
                        self.get_logger().info("Arm enabled message received from teensy, enabling arm node")

                elif(source_signal.value == SubsystemsIDs.SCIENCE):
                    self.__last_science_command = self.__curr_science_command
                    self.__curr_science_command = can_message.signals["timestamp"].value

                    # only set science enabled if it is not already
                    if(not self.__science_teensy_enabled):
                        self.__science_teensy_enabled = True
                        self.__science_teensy_enable_publisher.publish(self.__pack_enable_message_to_teensy(CAN_MESSAGE_IDS.ENABLE_SCIENCE, True))
                        self.get_logger().info("Science command received, enabling science teensy")
                    elif(can_message.signals["enabled"].value == 1 and not self.__science_node_enabled):
                        self.__science_node_enabled = True
                        self.__science_node_enable_publisher.publish(Bool(data=True))
                        self.get_logger().info("Science enabled message received from teensy, enabling science node")


            except Exception as e:
                self.get_logger().error(f"Failed to decode source signal from heartbeat message: {e}")
                return

    def __pack_enable_message_to_teensy(self, message_id: CAN_MESSAGE_IDS, enabled: bool) -> Can | None:
        if message_id not in TEENSY_CAN_MESSAGES:
            self.get_logger().error(f"Message ID {message_id} not found in TEENSY_CAN_MESSAGES")
            return None
        
        message = TEENSY_CAN_MESSAGES[message_id]
        message.signals["enable"].set_value(enabled)
        can_packet = TeensyCommunication.encode_can_message(message)
        if can_packet is not None:
            return can_packet
        else:
            self.get_logger().error(f"Failed to encode CAN message for {message.name}")

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-Stop message received, shutting down ...")

        rclpy.shutdown()

    def __on_auto_state_received(self, msg: UInt8):
        self.__auto_state_publisher.publish(UInt8(data=msg.data))

    def run(self):
        self.get_logger().info("starting master node ...")
        rclpy.spin(self)


def main():
    rclpy.init()
    master = Master()
    try:
        master.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        master.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
