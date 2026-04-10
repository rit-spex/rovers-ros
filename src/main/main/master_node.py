#!/usr/bin/env python3

from itertools import tee
import can
from numpy import int32, uint16

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
from constants.CAN_constants import (
    CAN_MESSAGE_IDS,
    TEENSY_CAN_MESSAGES,
    ArmState,
    ArmDirection,
    Subsystems_Names,
    SubsystemsIDs,
)
from constants.CAN_structs import Signal, Message
import time

# delay start up to ensure everything else is up and running before the master starts sending messages to the teensy
DELAY_STARTUP_SEC = 5  # seconds

timeout_duration = 1  # seconds


class Heartbeat_Manager:

    __master_node: Node

    __subsystem: str
    __message_id: CAN_MESSAGE_IDS

    __teensy_enabled_pub: rclpy.publisher.Publisher
    __teensy_enabled: bool  # we wait until the science teensy sends a heartbeat message before we consider the science subsystem enabled

    __node_enabled_pub: rclpy.publisher.Publisher
    __node_enabled: bool  # wait until teensy knows it is enabled before we consider the science node enabled

    __last_timestamp: UInt16
    __curr_timestamp: UInt16

    def __init__(
        self,
        teensy_enable_pub: rclpy.publisher.Publisher,
        node_enable_pub: rclpy.publisher.Publisher,
        subsystem: str,
        message_id: CAN_MESSAGE_IDS,
        master_node: Node,
    ) -> None:
        self.__master_node = master_node

        self.__subsystem = subsystem
        self.__message_id = message_id

        self.__teensy_enabled_pub = teensy_enable_pub
        self.__teensy_enabled = False

        self.__node_enabled_pub = node_enable_pub
        self.__node_enabled = False

        self.__last_timestamp = UInt16()
        self.__curr_timestamp = UInt16()

    def __pack_enable_message_to_teensy(
        self, message_id: CAN_MESSAGE_IDS, enabled: bool
    ) -> Can | None:
        if message_id not in TEENSY_CAN_MESSAGES:
            self.__master_node.get_logger().error(
                f"Message ID {message_id} not found in TEENSY_CAN_MESSAGES"
            )
            return None

        message = TEENSY_CAN_MESSAGES[message_id]
        message.signals["enable"].set_value(enabled)
        can_packet = TeensyCommunication.encode_can_message(message)
        if can_packet is not None:
            return can_packet
        else:
            self.__master_node.get_logger().error(
                f"Failed to encode CAN message for {message.name}"
            )

    def set_node_enabled(self, value: bool):
        # only send message if value is changed
        if self.__node_enabled != value:
            self.__node_enabled = value
            self.__node_enabled_pub.publish(Bool(data=value))

    def set_teensy_enabled(self, value: bool):
        # only send message if value is changed
        if self.__teensy_enabled != value:
            self.__teensy_enabled = value
            self.__teensy_enabled_pub.publish(
                self.__pack_enable_message_to_teensy(self.__message_id, value)
            )

    def check_timeout(self):
        # check for command timeout
        if self.__teensy_enabled:
            if self.__curr_timestamp == self.__last_timestamp:
                self.__master_node.get_logger().info(
                    f"{self.__subsystem} command timeout detected, disabling {self.__subsystem}"
                )

                self.set_node_enabled(False)
                self.set_teensy_enabled(False)

            else:
                self.__last_timestamp = self.__curr_timestamp

    def on_teensy_enable_message(self, enabled: bool, timestamp):

        # kill all messages with timestamp being the same
        if timestamp == self.__curr_timestamp:
            return

        self.__last_timestamp = self.__curr_timestamp
        self.__curr_timestamp = timestamp

        # check if the teensy is disabled and our nodes are disabled
        # this should be used when it first connects
        if not self.__teensy_enabled and not enabled:
            self.set_teensy_enabled(True)
            self.__master_node.get_logger().info(
                f"{self.__subsystem} status received, enabling {self.__subsystem} teensy"
            )

        # this should occur after the teensy is enabled and we haven't enabled the internal nodes yet
        # this is step two of the process
        elif enabled and not self.__node_enabled:
            self.set_node_enabled(True)
            self.__master_node.get_logger().info(
                f"{self.__subsystem} enabled message received from teensy, enabling {self.__subsystem} node"
            )

        # this is two turn off all everything if the teensy disabled
        elif not enabled and self.__node_enabled:
            self.set_node_enabled(False)
            self.__teensy_enabled = False
            self.__master_node.get_logger().info(
                f"{self.__subsystem} disable message received from teensy, disabling {self.__subsystem} node and teensy"
            )


class Master(Node):

    __estop_publisher: rclpy.publisher.Publisher

    __chassis_heartbeat_manager: Heartbeat_Manager
    __arm_heartbeat_manager: Heartbeat_Manager
    __science_heartbeat_manager: Heartbeat_Manager

    __auto_state_publisher: rclpy.publisher.Publisher

    __ros_teensy_publisher: (
        rclpy.publisher.Publisher
    )  # This is sent to the teensy to tell it that the ROS system is alive

    # basestation heartbeat tracking
    __last_basestation_heartbeat_time: UInt16
    __current_basestation_heartbeat_time: UInt16
    __basestation_recieved_heartbeat: bool

    def __init__(self):
        super().__init__("Master")
        self.__last_basestation_heartbeat_time = UInt16()
        self.__current_basestation_heartbeat_time = UInt16()
        self.__basestation_recieved_heartbeat = False

        # delay start up to ensure everything else is up and running before the master starts sending messages to the teensy
        time.sleep(DELAY_STARTUP_SEC)

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

        chassis_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/CHASSIS/ENABLED",
            qos_profile=10,
        )

        chassis_teensy_enable_publisher = self.create_publisher(
            msg_type=Can,
            topic=TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_CHASSIS].topic_name,
            qos_profile=10,
        )

        self.__chassis_heartbeat_manager = Heartbeat_Manager(
            chassis_teensy_enable_publisher,
            chassis_node_enable_publisher,
            "Chassis",
            CAN_MESSAGE_IDS.ENABLE_CHASSIS,
            self,
        )

        arm_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/ARM/ENABLED",
            qos_profile=10,
        )

        arm_teensy_enable_publisher = self.create_publisher(
            msg_type=Can,
            topic=TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_ARM].topic_name,
            qos_profile=10,
        )

        self.__arm_heartbeat_manager = Heartbeat_Manager(
            arm_teensy_enable_publisher,
            arm_node_enable_publisher,
            "Arm",
            CAN_MESSAGE_IDS.ENABLE_ARM,
            self,
        )

        science_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/SCIENCE/ENABLED",
            qos_profile=10,
        )

        science_teensy_enable_publisher = self.create_publisher(
            msg_type=Can,
            topic=TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_SCIENCE].topic_name,
            qos_profile=10,
        )

        self.__science_heartbeat_manager = Heartbeat_Manager(
            science_teensy_enable_publisher,
            science_node_enable_publisher,
            "Science",
            CAN_MESSAGE_IDS.ENABLE_SCIENCE,
            self,
        )

        self.__ros_teensy_publisher = self.create_publisher(
            msg_type=Can,
            topic=TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ROS_HEARTBEAT].topic_name,
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
        for message_id, message in TEENSY_CAN_MESSAGES.items():
            if message.subsystem == Subsystems_Names.GENERAL and message.isforJetson:
                self.create_subscription(
                    msg_type=Can,
                    topic=message.topic_name,
                    callback=self.__on_new_CAN_message_received,
                    qos_profile=10,
                )

    def __check_timeout(self) -> None:
        # check for basestation heartbeat timeout
        if self.__basestation_recieved_heartbeat:
            if (
                self.__current_basestation_heartbeat_time
                == self.__last_basestation_heartbeat_time
            ):
                self.get_logger().info("Heartbeat timeout detected")
                estop_msg = Bool()
                estop_msg.data = True
                self.__estop_publisher.publish(estop_msg)
            else:
                self.__last_basestation_heartbeat_time = (
                    self.__current_basestation_heartbeat_time
                )

        # check for teensy timeout
        self.__chassis_heartbeat_manager.check_timeout()
        self.__arm_heartbeat_manager.check_timeout()
        self.__science_heartbeat_manager.check_timeout()

        # send heartbeat message to teensy
        ros_heartbeat = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ROS_HEARTBEAT]
        ros_heartbeat.signals["timestamp"].set_value(
            int32((self.get_clock().now().nanoseconds // 1_000_000) % 100000000)
        )  # set timestamp to current time in milliseconds

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
        if can_message.id == CAN_MESSAGE_IDS.E_STOP:
            self.get_logger().info(
                "E-STOP message received from can_comms, forwarding ..."
            )
            estop_msg = Bool()
            estop_msg.data = True
            self.__estop_publisher.publish(estop_msg)
        elif can_message.id == CAN_MESSAGE_IDS.TEENSY_HEARTBEAT:
            try:
                source_signal = can_message.signals["source"]
                # self.get_logger().info(f"Source signal received: {can_message.toString()}")
                if source_signal.value == SubsystemsIDs.CHASSIS:
                    self.__chassis_heartbeat_manager.on_teensy_enable_message(
                        can_message.signals["enabled"].value,
                        can_message.signals["timestamp"].value,
                    )

                elif source_signal.value == SubsystemsIDs.ARM:
                    self.__arm_heartbeat_manager.on_teensy_enable_message(
                        can_message.signals["enabled"].value,
                        can_message.signals["timestamp"].value,
                    )

                elif source_signal.value == SubsystemsIDs.SCIENCE:
                    self.__science_heartbeat_manager.on_teensy_enable_message(
                        can_message.signals["enabled"].value,
                        can_message.signals["timestamp"].value,
                    )

            except Exception as e:
                self.get_logger().error(
                    f"Failed to decode source signal from heartbeat message: {e}"
                )
                return
            
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
