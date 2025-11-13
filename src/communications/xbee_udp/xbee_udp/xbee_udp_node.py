# ros imports
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

# TODO: change to custom type to use unsigned ints
from std_msgs.msg import UInt8MultiArray
# from custom_interfaces.msg import UDPPacket

import socket as skt
from socket import socket
import errno

class xbee_udp(Node):
    __address: str
    __port: int
    __socket: socket
    __buffer_size: int
    __publisher: Publisher

    def __init__(self) -> None:
        super().__init__("xbee_udp_node")

        self.__address = "127.0.0.1"
        self.__port = 5005
        self.__socket = socket(skt.AF_INET, skt.SOCK_DGRAM)
        self.__buffer_size = 1024
        self.__publisher = self.create_publisher(UInt8MultiArray, "/XBEE/MESSAGES", 10)

    def read_data(self, buffer_size: int) -> list[int] | None:
        try:
            data = self.__socket.recvfrom(buffer_size)
            self.get_logger().info(f"got data :)")

        except Exception as e:
            err = e.args
            # only error out if it wasn't from non-blocking
            if(err[0] == errno.EWOULDBLOCK):
                return []
            else:
                self.get_logger().info(f"failed to receive data: {e}")
                return []
        return list(data[0])

    def publish_data(self, data: list[int]) -> None:
        msg = UInt8MultiArray()
        msg.data = data
        self.__publisher.publish(msg)

    def run(self) -> None:
        self.get_logger().info("starting xbee udp socket connection...")
        self.__socket.bind((self.__address, self.__port))

        # make it so it will not stop the code when requesting a read
        self.__socket.setblocking(False)

        while True:
            data = self.read_data(self.__buffer_size)
            if data is None:
                self.get_logger().info("client disconnected")
                break
            if len(data) == 0:
                continue
            self.publish_data(data)
        rclpy.spin(self)

        # self.signal_estop()


def main():
    rclpy.init()
    udp = xbee_udp()
    udp.run()


if __name__ == "__main__":
    main()
