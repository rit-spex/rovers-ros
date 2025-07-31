import socket as skt
from socket import socket

from rclpy import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Int8MultiArray

class Udp(Node):
    __ip: str
    __port: int
    __socket: socket
    __buffer_size: int
    __publisher: Publisher

    def __init__(self) -> None:
        super().__init__("udp")

        self.__ip = "127.0.0.1"
        self.__port = 5000
        self.__socket = socket(skt.AF_INET, skt.SOCK_DGRAM)
        self.__buffer_size = 1024
        self.__publisher = self.create_publisher(Int8MultiArray, "/UDP/MESSAGES", 10)

        self.run()

    def read_data(self) -> list[int]:
        data, _ = self.__socket.recvfrom(self.__buffer_size)
        return list(data)

    def publish_data(self, data: list[int]) -> None:
        msg = Int8MultiArray()
        msg.data = data
        self.__publisher.publish(msg)

    def run(self) -> None:
        self.get_logger().info("starting socket connection...")

        self.__socket.bind((self.__ip, self.__port))

        while True:
            data = self.read_data()
            self.publish_data(data)
