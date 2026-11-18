import os
import can
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from custom_interfaces.msg import Can

from constants.CAN_constants import (
    TEENSY_CAN_MESSAGES,
    CAN_MESSAGE_IDS,
    SAFE_STOP_MESSAGE_IDS,
    TEENSY_ESTOP_TOPIC,
)
from constants.can_encoding import TeensyCommunication

import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool

INTERFACE = "socketcan"
CHANNEL = "can0"
BIT_RATE = 1000000  # 1 Mbps
ERROR_THRESHOLD = 5  # Number of errors before resetting the CAN network
CAN_ERROR_TOPIC = "can_err"


class CAN(Node):

    # Outgoing CAN topic subscriptions. They stay registered during E-STOP;
    # send_msg() ignores commands while the latch is active.
    __can_tx_subscriptions: dict[CAN_MESSAGE_IDS, rclpy.subscription.Subscription]
    __notifier: can.Notifier | None

    ERROR_THRESHOLD = 5  # Number of errors before resetting the CAN network
    __error_count = 0  # Current number of errors

    def __init__(self) -> None:
        super().__init__("CAN_node")
        self.__estop = False
        self.__error_count = 0
        self.reset_network()

        can.rc["interface"] = "socketcan"
        can.rc["bitrate"] = BIT_RATE

        # Do not loop our own outgoing frames back as hardware feedback.
        # E_STOP shares an ID in both directions, so echo would look like a
        # Teensy-side E-STOP.
        self.bus = can.Bus(
            CHANNEL, INTERFACE, bitrate=BIT_RATE, receive_own_messages=False
        )
        self.__notifier = can.Notifier(self.bus, [JETSON_LISTENER(self)])

        self.__can_tx_subscriptions = {}

        # This is for messages coming in
        for id, message in TEENSY_CAN_MESSAGES.items():
            if message.isforJetson:
                continue

            self.__can_tx_subscriptions[id] = self.create_subscription(
                msg_type=Can,
                topic=message.topic_name,
                callback=self.send_msg,
                qos_profile=10,
            )

        self.create_subscription(
            msg_type=Can,
            topic=CAN_ERROR_TOPIC,
            callback=self.__on_can_error,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

    def reset_network(self):
        self.get_logger().info("resetting the can network...")
        os.system("./src/communications/can_comms/can_comms/reset_can.zsh")

    def __on_estop_received(self, msg: Bool):
        # /ESTOP is level-triggered. Latch outbound commands while active,
        # send one round of safe defaults, and keep the node alive so it can
        # resume when E-STOP clears.
        if msg.data:
            if self.__estop:
                return
            self.__estop = True
            self.get_logger().info(
                "E-STOP received, sending safe CAN defaults")
            self.__send_safe_stop()
            try:
                self.bus.flush_tx_buffer()
            except NotImplementedError:
                # SocketCAN transmits synchronously and does not implement
                # flush_tx_buffer; without this guard the E-STOP callback
                # raises and takes the whole node down.
                pass
        else:
            if not self.__estop:
                return
            self.__estop = False
            self.__send_estop_clear()
            self.get_logger().info(
                "E-STOP cleared, CAN output re-enabled")

    def __on_can_error(self, msg: Can):
        self.__error_count += 1
        self.get_logger().info(
            f"CAN error received: 0x{msg.id:03x} "
            f"({self.__error_count}/{ERROR_THRESHOLD})"
        )
        if self.__error_count >= ERROR_THRESHOLD:
            self.get_logger().error(
                f"CAN error threshold reached ({self.__error_count} errors), "
                "resetting the CAN network"
            )
            self.reset_network()
            self.__error_count = 0

    def __send_safe_stop(self, include_estop: bool = True):
        """Send only outgoing messages whose defaults are safe stop values.

        On shutdown ``include_estop`` is false so stopping the process does
        not leave the Teensy latched in its own E-STOP state.
        """
        for message_id in SAFE_STOP_MESSAGE_IDS:
            if not include_estop and message_id == CAN_MESSAGE_IDS.E_STOP:
                continue
            message = TEENSY_CAN_MESSAGES[message_id]
            message.reset()
            can_packet = TeensyCommunication.encode_can_message(message)
            if can_packet is not None:
                self.__send_can(can_packet)

    def __send_estop_clear(self):
        """Tell the Teensy the E-STOP condition has cleared."""
        message = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.E_STOP]
        message.signals["E_STOP"].set_value(0)
        can_packet = TeensyCommunication.encode_can_message(message)
        if can_packet is not None:
            self.__send_can(can_packet)

    def send_msg(self, msg: Can):
        if self.__estop:
            return
        self.__send_can(msg)

    def __send_can(self, msg: Can):
        self.get_logger().debug(
            f"ID {msg.id} ({TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS(msg.id)].name}): {msg.buf}"
        )
        bus_msg = can.Message(
            arbitration_id=msg.id, data=list(msg.buf), is_extended_id=False
        )
        try:
            self.bus.send(bus_msg)
        except Exception as e:
            self.get_logger().info(f"Failed to send message: {e}")

    def destroy_node(self):
        # Best-effort actuator safe state while the bus is still open.
        try:
            self.__send_safe_stop(include_estop=False)
        except Exception:
            pass
        if self.__notifier is not None:
            try:
                self.__notifier.stop()
            except Exception:
                pass
            self.__notifier = None
        try:
            self.bus.shutdown()
        except Exception:
            pass
        return super().destroy_node()

    def run(self):
        rclpy.spin(self)


class JETSON_LISTENER(can.Listener):
    __node: Node

    # Stored as Publishers[message_id]
    __publishers: dict[int, rclpy.publisher.Publisher]

    def __init__(self, node) -> None:
        super().__init__()
        self.__node = node

        self.__publishers = {}

        # E_STOP is bidirectional on the same CAN ID. Incoming frames are
        # republished here so master_node can latch a Teensy-side E-STOP.
        self.__estop_publisher = self.__node.create_publisher(
            msg_type=Can, topic=TEENSY_ESTOP_TOPIC, qos_profile=10
        )

        # This is for messages coming in
        for id, message in TEENSY_CAN_MESSAGES.items():
            if not message.isforJetson:
                continue

            self.__publishers[id] = self.__node.create_publisher(
                msg_type=Can, topic=message.topic_name, qos_profile=10
            )

        self.__error_publisher = self.__node.create_publisher(
            msg_type=Can, topic=CAN_ERROR_TOPIC, qos_profile=10
        )

    def on_message_received(self, msg: can.Message) -> None:
        if msg.is_error_frame:
            self.error_frame_publisher.publish(msg)
        elif msg.arbitration_id in self.__publishers:
            buf = [0] * 8  # Initialize buffer with 8 zeros
            for i in range(msg.dlc):
                buf[i] = msg.data[i]

    @staticmethod
    def __buffer(msg: can.Message) -> list[int]:
        """Copy a frame payload into the fixed 8-byte ROS CAN layout."""
        buf = [0] * 8
        for i in range(min(msg.dlc, 8)):
            buf[i] = msg.data[i]
        return buf

    def on_error(self, exc: Exception) -> bool:
        """Keep the notifier alive when the interface briefly goes down.

        Resetting can0 makes ``recv`` raise ``CanOperationError``.  Returning
        True tells python-can the error was handled so the reader thread keeps
        running until the link is back up.
        """
        self.__node.get_logger().warning(f"CAN receive error: {exc}")
        return True

    def on_message_received(self, msg: can.Message) -> None:
        # Error frames are not addressed to a motor; route them to the bus
        # health path before the E_STOP/ID lookup below.
        if msg.is_error_frame:
            self.__error_publisher.publish(
                Can(id=msg.arbitration_id, buf=self.__buffer(msg))
            )
            return

        if msg.arbitration_id == CAN_MESSAGE_IDS.E_STOP:
            self.__estop_publisher.publish(
                Can(id=msg.arbitration_id, buf=self.__buffer(msg))
            )
            self.__node.get_logger().info(
                "Hardware E-STOP frame received from CAN"
            )
            return

        if msg.arbitration_id in self.__publishers:
            ros_msg = Can(
                id=msg.arbitration_id,
                buf=self.__buffer(msg),
            )
            self.__node.get_logger().debug(
                f"ID {msg.arbitration_id} ({TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS(msg.arbitration_id)].name}): {msg.data}"
            )

            self.__publishers[msg.arbitration_id].publish(ros_msg)
        else:
            # Optional: Log a warning instead of crashing
            self.__node.get_logger().debug(
                f"Received unknown CAN ID: {msg.arbitration_id}"
            )


def main():
    rclpy.init()
    can = CAN()
    try:
        can.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        can.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
