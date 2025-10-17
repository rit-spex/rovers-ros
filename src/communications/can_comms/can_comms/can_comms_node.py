import os
import time
import can
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import CanFD, Can
from constants.CAN_Constants import TOPICS
import rclpy.publisher
import rclpy.subscription

INTERFACE = "socketcan"
CHANNEL = "can0"
BIT_RATE = 500000


class CAN(Node):
    __subscriptions: list[rclpy.subscription.Subscription] = []

    def __init__(self) -> None:
        super().__init__("CAN_node")
        self.reset_network()

        can.rc["interface"] = "socketcan"
        can.rc["bitrate"] = 500000

        self.bus = can.Bus(
            CHANNEL, INTERFACE, bitrate=BIT_RATE, receive_own_messages=True
        )
        can.Notifier(self.bus, [JETSON_LISTENER(self)])

        for message_id in TOPICS.keys():
            self.__subscriptions.append(
                self.create_subscription(
                    msg_type=Can,
                    topic=f"/CAN/TX/{TOPICS[message_id]['name']}",
                    callback=self.send_msg,
                    qos_profile=10,
                )
            )

        self.run()

    def reset_network(self):
        self.get_logger().info("resetting the can network...")
        os.system("./src/communications/can_comms/can_comms/reset_can.zsh")

    def send_msg(self, msg):
        self.get_logger().info(f"ID {msg.id} ({TOPICS[msg.id]['name']}): {msg.buf}")
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

    def __init__(self, node) -> None:
        super().__init__()
        self.__node = node

    def on_message_received(self, msg: can.Message) -> None:
        pass


def main():
    rclpy.init()
    can = CAN()

if __name__ == "__main__":
    main()
