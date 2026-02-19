import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.subscription

from custom_interfaces.msg import Can
from constants.CAN_Constants import TOPICS

import socket as skt
from socket import socket
import errno

# this is used to package the message in certain formats
from struct import pack

class CanUdp(Node):
    __address: str
    __recv_port: int
    __send_port: int
    __socket: socket
    __subscriptions: list[rclpy.subscription.Subscription]

    def __init__(self) -> None:
        super().__init__("CAN_udp_node")

        self.__address = "127.0.0.1"
        self.__recv_port = 8010
        self.__send_port = 8000
        self.__socket = socket(skt.AF_INET, skt.SOCK_DGRAM)
        self.__subscriptions = []
        # can.Notifier(self.bus, [JETSON_LISTENER(self)])

        for message_id in TOPICS.keys():
            self.__subscriptions.append(
                self.create_subscription(
                    msg_type=Can,
                    topic=f"/CAN/TX/{TOPICS[message_id]['name']}",
                    callback=self.send_msg,
                    qos_profile=10,
                )
            )

    def send_msg(self, msg: Can):
        """
        pack and send the message to the teensy
        """
        self.get_logger().info(f"ID {msg.id} ({TOPICS[msg.id]['name']}): {msg.buf}")

        # make sure the message is at most 8 bytes long, otherwise throw an error
        if len(msg.buf) > 8:
            raise ValueError(
                f"CAN message length must be <= 8 bytes, got {len(msg.buf)} for ID {msg.id}"
            )

        # details how to pack the data, i = 4 bytes, B = unsigned byte
        pack_format = ">iB" + "B" * len(msg.buf)

        # package the message with the format and the id and data
        udp_packet = pack(pack_format, msg.id, len(msg.buf), *msg.buf)

        try:
            self.__socket.sendto(udp_packet, (self.__address, self.__send_port))
        except OSError as e:
            if e.errno == errno.EWOULDBLOCK:
                return
            self.get_logger().error(f"failed to send can udp packet: {e}")

    def run(self):
        # start the socket so we can send and recv messages
        self.get_logger().info("starting can udp socket connection...")
        self.__socket.bind((self.__address, self.__recv_port))

        # make it so it will not stop the code when requesting a read
        self.__socket.setblocking(False)

        try:
            rclpy.spin(self)
        finally:
            self.__socket.close()

def main():
    rclpy.init()
    can_udp = CanUdp()
    try:
        can_udp.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        can_udp.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
