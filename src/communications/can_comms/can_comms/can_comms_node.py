import os
import can
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import CanFD, Can

from constants.CAN_constants import TEENSY_CAN_MESSAGES, CAN_MESSAGE_IDS
from constants.can_encoding import TeensyCommunication

import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16

INTERFACE = "socketcan"
CHANNEL = "can0"
BIT_RATE = 1000000  # 1 Mbps


class CAN(Node):

    # This is needed as a var to shut it down early during E-stop
    __can_tx_subscriptions: dict[CAN_MESSAGE_IDS, rclpy.subscription.Subscription]

    def __init__(self) -> None:
        super().__init__("CAN_node")
        self.reset_network()

        can.rc["interface"] = "socketcan"
        can.rc["bitrate"] = BIT_RATE

        self.bus = can.Bus(
            CHANNEL, INTERFACE, bitrate=BIT_RATE, receive_own_messages=True
        )
        can.Notifier(self.bus, [JETSON_LISTENER(self)])

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
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

    def reset_network(self):
        self.get_logger().info("resetting the can network...")
        os.system("./src/communications/can_comms/can_comms/reset_can.zsh")

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, shutting down can_comms_node")

        # First turn off the subscriptions so no new messages can be sent
        for subscription in self.__can_tx_subscriptions.values():
            self.destroy_subscription(subscription)

        self.get_logger().info(
            "CAN_node: sending default messages before shutting down"
        )
        # clear out all of the messages with default values
        for message_id, message in TEENSY_CAN_MESSAGES.items():
            # Set all of the value to default
            message.reset()
            can_packet = TeensyCommunication.encode_can_message(message)
            if can_packet is not None:
                self.send_msg(can_packet)

        # Flush all of the messages before shutting down
        self.bus.flush_tx_buffer()
        self.bus.shutdown()
        rclpy.shutdown()

    def send_msg(self, msg: Can):
        self.get_logger().info(
            f"ID {msg.id} ({TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS(msg.id)].name}): {msg.buf}"
        )
        bus_msg = can.Message(
            arbitration_id=msg.id, data=list(msg.buf), is_extended_id=False
        )
        try:
            self.bus.send(bus_msg)
        except Exception as e:
            self.get_logger().info(f"Failed to send message: {e}")

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

        # This is for messages coming in
        for id, message in TEENSY_CAN_MESSAGES.items():
            if not message.isforJetson:
                continue

            self.__publishers[id] = self.__node.create_publisher(
                msg_type=Can, topic=message.topic_name, qos_profile=10
            )

    def on_message_received(self, msg: can.Message) -> None:
        if msg.arbitration_id in self.__publishers:
            buf = [0] * 8  # Initialize buffer with 8 zeros
            for i in range(msg.dlc):
                buf[i] = msg.data[i]

            ros_msg = Can(
                id=msg.arbitration_id,
                buf=buf,
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
    can.run()


if __name__ == "__main__":
    main()
