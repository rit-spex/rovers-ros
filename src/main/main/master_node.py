#!/usr/bin/env python3

from numpy import int32

# ros imports
import rclpy
import rclpy.logging
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
import rclpy.publisher
import rclpy.subscription
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, UInt16
from custom_interfaces.msg import Can

from constants.CommandCodes import CONSTANTS
from constants.can_encoding import TeensyCommunication
from constants.CAN_constants import (
    CAN_MESSAGE_IDS,
    TEENSY_CAN_MESSAGES,
    TEENSY_ESTOP_TOPIC,
    ArmState,
    ArmDirection,
    Subsystems_Names,
    SubsystemsIDs,
)
from constants.CAN_structs import Signal, Message
import time

# delay start up to ensure everything else is up and running before the master starts sending messages to the teensy
DELAY_STARTUP_SEC = 5 # seconds

# How often the master checks command and heartbeat timeouts [s].
timeout_duration = 1  # seconds

# The basestation sends a heartbeat every 200 ms.  The effective detection
# window is 1 s: latch after 0.9 s of silence, and the 0.1 s check granularity
# brings the worst-case detection to 1.0 s.
HEARTBEAT_DETECTION_SEC = 1.0
HEARTBEAT_CHECK_PERIOD_SEC = 0.1

# A check timer that fires this much later than scheduled means the process
# was descheduled (a stalled VM or a loaded host does this for seconds at a
# time).  The recorded silence then includes time this node could not run, so
# it is not evidence that the basestation link died.  Skip those samples
# instead of flapping E-STOP; a real outage still latches one tick later.
HEARTBEAT_LATE_CHECK_SEC = 0.5

# Raw teleop/XBee frames are high-rate and loss tolerant.  Best-effort QoS
# keeps a slow consumer from blocking the transport and stalling heartbeats.
XBEE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

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
    __last_basestation_heartbeat_rx: float
    __received_basestation_heartbeat: bool
    __last_heartbeat_check_rx: float
    __heartbeat_silence_limit_sec: float
    __heartbeat_estop: bool
    __hardware_estop: bool
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
        self.__last_basestation_heartbeat_rx = 0.0
        self.__received_basestation_heartbeat = False
        self.__last_heartbeat_check_rx = time.monotonic()
        self.declare_parameter("heartbeat_timeout_sec",
                               HEARTBEAT_DETECTION_SEC)
        effective_window = max(
            2 * HEARTBEAT_CHECK_PERIOD_SEC,
            float(self.get_parameter("heartbeat_timeout_sec").value))
        self.__heartbeat_silence_limit_sec = (
            effective_window - HEARTBEAT_CHECK_PERIOD_SEC)
        # Fail safe: stay latched until the basestation proves it is alive.
        self.__heartbeat_estop = True
        self.__hardware_estop = False

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
            topic="/ROVER/TELEMETRY/AUTO_STATE",
            qos_profile=10,
        )

        self.__chassis_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/ROVER/TELEMETRY/SUBSYSTEM/CHASSIS_ENABLED",
            qos_profile=10,
        )

        self.__chassis_teensy_enable_publisher = self.create_publisher(
            msg_type=Can,
            topic= TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_CHASSIS].topic_name,
            qos_profile=10,
        )

        self.__arm_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/ARM/STATE/ENABLED",
            qos_profile=10,
        )

        self.__arm_teensy_enable_publisher = self.create_publisher(
            msg_type=Can,
            topic= TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_ARM].topic_name,
            qos_profile=10,
        )

        self.__science_node_enable_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/ROVER/TELEMETRY/SUBSYSTEM/SCIENCE_ENABLED",
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
            topic="/BASESTATION/"
            + CONSTANTS.QUIT.NAME
            + "/"
            + CONSTANTS.QUIT.QUIT_MESSAGE,
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
        # The decoded heartbeat topic passes through basestation_node.  Also
        # watch the raw transport so a stalled decode bridge cannot look like
        # a dead operator link.
        self.create_subscription(
            msg_type=UInt8MultiArray,
            topic="/XBEE/MESSAGES/RX",
            callback=self.__on_xbee_message_received,
            qos_profile=XBEE_QOS,
        )
        self.create_subscription(
            msg_type=UInt8,
            topic="/BASESTATION/auto_state/auto_state",
            callback=self.__on_auto_state_received,
            qos_profile=10,
        )

        # Teensy-side hardware E-STOP. E_STOP shares one CAN ID in both
        # directions; the transport republishes inbound frames here.
        self.create_subscription(
            msg_type=Can,
            topic=TEENSY_ESTOP_TOPIC,
            callback=self.__on_new_CAN_message_received,
            qos_profile=10,
        )

        self.create_timer(
            timer_period_sec=timeout_duration,
            callback=self.__check_timeout,
        )
        self.create_timer(
            timer_period_sec=HEARTBEAT_CHECK_PERIOD_SEC,
            callback=self.__check_heartbeat,
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

        # Publish the fail-safe state before the startup delay so other nodes
        # latch immediately instead of waiting for the first timer tick.
        self.__publish_estop()
        # delay start up to ensure everything else is up and running before the master starts sending messages to the teensy
        time.sleep(DELAY_STARTUP_SEC)
        self.get_logger().info(
            "E-STOP latched until the basestation heartbeat is received")
        self.get_logger().info(
            "Basestation heartbeat window: %.1f s"
            % (self.__heartbeat_silence_limit_sec
               + HEARTBEAT_CHECK_PERIOD_SEC))

    def __check_heartbeat(self) -> None:
        """Latch the heartbeat E-stop after a full grace window of silence."""
        now = time.monotonic()
        since_last_check = now - self.__last_heartbeat_check_rx
        self.__last_heartbeat_check_rx = now
        if not self.__received_basestation_heartbeat:
            return
        # A late timer means this process itself was starved; the silence we
        # measured is our own scheduling delay, not evidence about the link.
        # Restart the grace window instead of latching on that invalid sample.
        if since_last_check > HEARTBEAT_LATE_CHECK_SEC:
            self.__last_basestation_heartbeat_rx = now
            return
        silent_for = now - self.__last_basestation_heartbeat_rx
        if (silent_for > self.__heartbeat_silence_limit_sec
                and not self.__heartbeat_estop):
            self.get_logger().warn(
                f"Basestation heartbeat timeout ({silent_for:.2f} s)")
            self.__heartbeat_estop = True
            self.__publish_estop()

    def __check_timeout(self) -> None:
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

        # Republish every tick so nodes that start after a transition still
        # see the current level, and so consumers re-arm after an E-STOP.
        self.__publish_estop()
        self.__publish_enable_levels()

    def __publish_estop(self) -> None:
        """Publish the current level-triggered E-STOP state."""
        self.__estop_publisher.publish(
            Bool(data=self.__heartbeat_estop or self.__hardware_estop))

    def __publish_enable_levels(self) -> None:
        """Republish the enabled state of each subsystem.

        The CAN transports drop outbound frames while ``/ESTOP`` is active,
        so an enable sent at the instant the stop clears can be swallowed.
        Republishing the level lets the arm/chassis nodes and the Teensy
        re-arm as soon as the latch is gone.
        """
        teensy_enables = (
            (self.__chassis_teensy_enabled,
             self.__chassis_teensy_enable_publisher,
             CAN_MESSAGE_IDS.ENABLE_CHASSIS),
            (self.__arm_teensy_enabled,
             self.__arm_teensy_enable_publisher,
             CAN_MESSAGE_IDS.ENABLE_ARM),
            (self.__science_teensy_enabled,
             self.__science_teensy_enable_publisher,
             CAN_MESSAGE_IDS.ENABLE_SCIENCE),
        )
        for enabled, publisher, message_id in teensy_enables:
            if not enabled:
                continue
            packet = self.__pack_enable_message_to_teensy(message_id, True)
            if packet is not None:
                publisher.publish(packet)

        node_enables = (
            (self.__chassis_node_enabled,
             self.__chassis_node_enable_publisher),
            (self.__arm_node_enabled, self.__arm_node_enable_publisher),
            (self.__science_node_enabled,
             self.__science_node_enable_publisher),
        )
        for enabled, publisher in node_enables:
            if enabled:
                publisher.publish(Bool(data=True))

    def __on_quit_received(self, msg: Bool):
        self.get_logger().info("Quit message received, forwarding e-stop ...")
        if msg.data:
            # A deliberate quit is treated like a lost heartbeat: a new
            # basestation session can clear it by sending a heartbeat.
            self.__heartbeat_estop = True
        # Never let a quit-clear override a latched hardware e-stop.
        self.__publish_estop()

    def __on_basestation_heartbeat_received(self, msg: UInt16):
        self.__mark_basestation_heartbeat()

    def __on_xbee_message_received(self, msg: UInt8MultiArray):
        """Use the raw transport heartbeat as an independent liveness path."""
        if not msg.data or msg.data[0] != CONSTANTS.HEARTBEAT.MESSAGE[0]:
            return
        self.__mark_basestation_heartbeat()

    def __mark_basestation_heartbeat(self):
        # Any heartbeat arrival proves the basestation is alive; the payload
        # is only a sequence stamp and may legitimately repeat.
        self.__received_basestation_heartbeat = True
        self.__last_basestation_heartbeat_rx = time.monotonic()
        if self.__heartbeat_estop:
            self.__heartbeat_estop = False
            # A hardware e-stop stays latched until the node restarts.
            if not self.__hardware_estop:
                self.get_logger().info(
                    "Heartbeat restored, clearing e-stop")
            self.__publish_estop()

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
            if can_message.signals["E_STOP"].value:
                if not self.__hardware_estop:
                    self.get_logger().info(
                        "Hardware E-STOP received; latching until restart")
                    self.__hardware_estop = True
            elif self.__hardware_estop:
                self.get_logger().info(
                    "Hardware E-STOP clear received; staying latched until restart")
            return
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

    def __on_auto_state_received(self, msg: UInt8):
        self.__auto_state_publisher.publish(UInt8(data=msg.data))

    def destroy_node(self):
        # Best effort: tell the rest of the stack to stop as we go down.
        try:
            self.__estop_publisher.publish(Bool(data=True))
        except Exception:
            pass
        return super().destroy_node()

    def run(self):
        self.get_logger().info("starting master node ...")
        rclpy.spin(self)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    master = None
    try:
        master = Master()
        master.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if master is not None:
            try:
                master.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
