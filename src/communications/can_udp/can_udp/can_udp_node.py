import rclpy
from rclpy.node import Node
import rclpy.subscription
import rclpy.publisher

from custom_interfaces.msg import Can
from constants.CAN_Constants import TEENSY_CAN_MESSAGES

import socket as skt
from socket import socket
import errno

# this is used to package the message in certain formats
from struct import *

class CAN_UDP(Node):
    __address: str
    __recv_port: int
    __send_port: int
    __socket: socket
    __buffer_size: int
    __publisher: rclpy.publisher.Publisher

    def __init__(self) -> None:
        super().__init__("CAN_udp_node")

        self.__address = "127.0.0.1"
        self.__recv_port = 8010
        self.__send_port = 8000
        self.__socket = socket(skt.AF_INET, skt.SOCK_DGRAM)
        self.__buffer_size = 1024
        # can.Notifier(self.bus, [JETSON_LISTENER(self)])

        # This is for messages coming in
        self.__publisher = self.create_publisher(
            msg_type=Can,
            topic="/CAN/RX",
            qos_profile=10,
        )

        # This is for messages going out
        self.create_subscription(
            msg_type=Can,
            topic="/CAN/TX",
            callback=self.send_msg,
            qos_profile=10,
        )

        self.run()

    def send_msg(self, msg):
        """
        pack and send the message to the teensy
        """
        self.get_logger().info(f"ID {msg.id} ({TEENSY_CAN_MESSAGES[msg.id].name}): {msg.buf}")

        # make sure the message is at most 8 bytes long, otherwise throw an error
        if(len(msg.buf) > 8):
            raise Exception

        # details how to pack the data, i = 4 bytes, B = unsigned byte
        pack_format = ">iB" + "B" * len(msg.buf)

        # package the message with the format and the id and data
        udp_packet = pack(pack_format, msg.id, len(msg.buf), *msg.buf)

        try:
            self.__socket.sendto(udp_packet, (self.__address, self.__send_port))
        except Exception as e:
            err = e.args
            # only error out if it wasn't from non-blocking
            if(err[0] == errno.EWOULDBLOCK):
                pass
            else:
                self.get_logger().info(f"failed to send can udp packet: {e}")

    def run(self):
        # start the socket so we can send and recv messages
        self.get_logger().info("starting can udp socket connection...")
        self.__socket.bind((self.__address, self.__recv_port))

        # make it so it will not stop the code when requesting a read
        self.__socket.setblocking(False)

        rclpy.spin(self)

def main():
    rclpy.init()
    can_udp = CAN_UDP()
    can_udp.run()

if __name__ == "__main__":
    main()
