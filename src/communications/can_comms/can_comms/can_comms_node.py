import os
import time
import can
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import CanFD, Can
from constants.CAN_Constants import TEENSY_CAN_MESSAGES
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16

INTERFACE = "socketcan"
CHANNEL = "can0"
BIT_RATE = 500000


class CAN(Node):

    def __init__(self) -> None:
        super().__init__("CAN_node")
        self.reset_network()

        can.rc["interface"] = "socketcan"
        can.rc["bitrate"] = 500000

        self.bus = can.Bus(
            CHANNEL, INTERFACE, bitrate=BIT_RATE, receive_own_messages=True
        )
        can.Notifier(self.bus, [JETSON_LISTENER(self)])

        self.create_subscription(
                msg_type=Can,
                topic=f"/CAN/TX",
                callback=self.send_msg,
                qos_profile=10,
            )

    def reset_network(self):
        self.get_logger().info("resetting the can network...")
        os.system("./src/communications/can_comms/can_comms/reset_can.zsh")

    def send_msg(self, msg):
        self.get_logger().info(f"ID {msg.id} ({TEENSY_CAN_MESSAGES[msg.id].name}): {msg.buf}")
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

    __publisher: rclpy.publisher.Publisher

    def __init__(self, node) -> None:
        super().__init__()
        self.__node = node
        self.__publisher = self.__node.create_publisher(
            msg_type=Can,
            topic=f"/CAN/RX",
            qos_profile=10,
        )

    def on_message_received(self, msg: can.Message) -> None:
        ros_msg = Can(
            id=msg.arbitration_id,
            buf=msg.data,
        )
        self.__publisher.publish(ros_msg)

def main():
    rclpy.init()
    can = CAN()
    can.run()

if __name__ == "__main__":
    main()
