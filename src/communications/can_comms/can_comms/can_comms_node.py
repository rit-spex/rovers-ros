import os
import time
import can
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import CanFD, Can
from constants.CAN_Constants import TOPICS
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16

INTERFACE = "socketcan"
CHANNEL = "can0"
BIT_RATE = 500000


class CAN(Node):
    __subscriptions: list[rclpy.subscription.Subscription] = []

    def __init__(self) -> None:
        super().__init__("CAN_node")
        self.bus = None

        self.reset_network()

        can.rc["interface"] = "socketcan"
        can.rc["bitrate"] = 500000

        try:
            self.bus = can.Bus(
                CHANNEL, INTERFACE, bitrate=BIT_RATE, receive_own_messages=True
            )
            can.Notifier(self.bus, [JETSON_LISTENER(self)])
        except Exception as exc:
            self.get_logger().warning(
                f"SocketCAN unavailable on {CHANNEL}; CAN comms disabled ({exc})"
            )
            self.bus = None

        for message_id in TOPICS.keys():
            self.__subscriptions.append(
                self.create_subscription(
                    msg_type=Can,
                    topic=f"/CAN/TX/{TOPICS[message_id]['name']}",
                    callback=self.send_msg,
                    qos_profile=10,
                )
            )

    def reset_network(self):
        self.get_logger().info("resetting the can network...")
        if os.geteuid() != 0:
            self.get_logger().warning(
                "Skipping CAN reset: root permissions required"
            )
            return
        os.system("./src/communications/can_comms/can_comms/reset_can.zsh")

    def send_msg(self, msg):
        if self.bus is None:
            return
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
    node = None
    try:
        node = CAN()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"can_comms node failed: {exc}")
        else:
            print(f"can_comms node failed before logger initialization: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
