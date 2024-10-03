import can
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import CanFD, Can
from constants.CAN_Constants import CHANNEL, TOPICS
import rclpy.publisher
import rclpy.subscription


class CAN(Node):
    __subscriptions = []

    def __init__(self) -> None:

        jetson = can.Bus(channel=CHANNEL.JETSON, interface="isCAN")
        can.Notifier(jetson, [JETSON_LISTENER(self)])

        for message_id in range(0, len(TOPICS)):
            self.__subscriptions.append(
                self.create_subscription(
                    Can, f"/CAN/TX/{TOPICS[message_id]['name']}", self.send_msg, 10
                )
            )
        self.run()

    def send_msg(self, msg):
        with can.Bus(msg.channel) as bus:
            bus_msg = can.Message(arbitration_id=msg.id, data=msg.buf)
            try:
                bus.send(bus_msg)
            except:
                self.get_logger().debug("Failed to send message")

    def run(self):
        rclpy.spin(self)


class JETSON_LISTENER(can.Listener):
    __node: Node

    def __init__(self, node) -> None:
        super().__init__()
        self.__node = node

    def on_message_received(self, msg: can.Message) -> None:
        ros_msg = Can()
        ros_msg.channel = TOPICS[msg.arbitration_id]["channel"]
        ros_msg.id = msg.arbitration_id
        ros_msg.buf = msg.data

        self.__node.create_publisher(
            Can, f"/CAN/RX/{TOPICS[msg.arbitration_id]['name']}", 10
        ).publish(ros_msg)


def main():
    can = CAN()


if __name__ == "__main__":
    main()
