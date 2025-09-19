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
        # can.rc['poll_interval'] = 0.01

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
        # os.system("sudo ifconfig can0 down")
        # os.system("sudo ip link set can0 up type can bitrate 500000")
        # os.system("sudo ip link set can0 up")
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
        # uncomment if you're brave enough
        # ros_msg = Can()
        # ros_msg.channel = TOPICS[msg.arbitration_id]["channel"]
        # ros_msg.id = msg.arbitration_id
        # ros_msg.buf = msg.data

        # self.__node.create_publisher(
        #     Can, f"/CAN/RX/{TOPICS[msg.arbitration_id]['name']}", 10
        # ).publish(ros_msg)
        pass


def main():
    # can.rc['baudrate'] = 500

    rclpy.init()
    can = CAN()
    # can.run()
    # bus = can.Bus(channel="can0", interface="socketcan")
    # for i in range(255):
    #     msg = can.Message(arbitration_id=0xc0ffee, data=[1, i, 0, 1, 3, 1, 4, 1], is_extended_id=False)
    #     bus.send(msg)

    # time.sleep(1)
    # bus.shutdown()


if __name__ == "__main__":
    main()
