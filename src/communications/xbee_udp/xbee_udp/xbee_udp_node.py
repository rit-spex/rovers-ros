# ros imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import UInt8MultiArray

import select
import socket as skt
from socket import socket

from constants.CommandCodes import CONSTANTS

# Drain a burst in one pass so the rover-side UDP queue cannot grow while
# the process is also publishing to ROS.
MAX_DRAIN_PER_PASS = 32


class xbee_udp(Node):
    __address: str
    __port: int
    __socket: socket
    __buffer_size: int
    __publisher: Publisher

    def __init__(self) -> None:
        super().__init__("xbee_udp_node")

        self.__address = CONSTANTS.COMMUNICATION.UDP_HOST
        self.__port = CONSTANTS.COMMUNICATION.UDP_ROVER_PORT
        self.__basestation_port = CONSTANTS.COMMUNICATION.UDP_BASESTATION_PORT
        self.__socket = socket(skt.AF_INET, skt.SOCK_DGRAM)
        self.__buffer_size = 1024
        self.__publisher = self.create_publisher(
            UInt8MultiArray, "/XBEE/MESSAGES/RX",
            QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(UInt8MultiArray, "/XBEE/MESSAGES/TX", self.send_msg, 10)

    def read_data(self, buffer_size: int) -> list[int]:
        try:
            data, addr = self.__socket.recvfrom(buffer_size)
            payload = list(data)
            self.get_logger().debug(
                f"received {len(payload)} bytes from {addr[0]}:{addr[1]}: "
                f"{data.hex(' ')}"
            )
        except Exception as e:
            self.get_logger().error(f"failed to receive data: {e}")
            return []
        return payload

    def send_msg(self, msg: UInt8MultiArray) -> None:
        """
        pack and send the message to the basestation
        """
        try:
            self.__socket.sendto(bytes(msg.data), (self.__address, self.__basestation_port))
            self.get_logger().debug(
                f"sent {len(msg.data)} bytes to {self.__address}:{self.__basestation_port}: "
                f"{bytes(msg.data).hex(' ')}"
            )
        except Exception as e:
            self.get_logger().error(f"failed to send data: {e}")

    def publish_data(self, data: list[int]) -> None:
        msg = UInt8MultiArray()
        msg.data = data
        self.__publisher.publish(msg)

    def run(self) -> None:
        self.get_logger().info("starting xbee udp socket connection...")
        self.__socket.setsockopt(
            skt.SOL_SOCKET, skt.SO_RCVBUF, 1 << 20)
        self.__socket.bind((self.__address, self.__port))

        try:
            while rclpy.ok():
                # Wait up to 10 ms for data instead of non-blocking spin.
                ready, _, _ = select.select([self.__socket], [], [], 0.01)
                if ready:
                    for _ in range(MAX_DRAIN_PER_PASS):
                        data = self.read_data(self.__buffer_size)
                        if not data:
                            break
                        self.publish_data(data)
                        more, _, _ = select.select(
                            [self.__socket], [], [], 0)
                        if not more:
                            break
                rclpy.spin_once(self, timeout_sec=0)
        finally:
            self.__socket.close()


def main():
    rclpy.init()
    udp = xbee_udp()
    try:
        udp.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        udp.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
