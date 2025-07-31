import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Int8MultiArray

from socket import socket


class Udp(Node):
    __ip: str
    __port: int
    __socket: socket
    __buffer_size: int
    __publisher: Publisher

    def __init__(self) -> None:
        super().__init__("udp")

        self.__ip = "127.0.0.1"
        self.__port = 5005
        self.__socket = socket()
        self.__buffer_size = 1024
        self.__publisher = self.create_publisher(Int8MultiArray, "/UDP/MESSAGES", 10)

        self.run()

    def read_data(self) -> list[int]:
        data = self.__socket.recv(self.__buffer_size)
        return list(data)

    def publish_data(self, data: list[int]) -> None:
        msg = Int8MultiArray()
        msg.data = data
        self.__publisher.publish(msg)

    def run(self) -> None:
        self.get_logger().info("starting socket connection...")

        self.get_logger().info("waiting for connection...")
        while True:
            try:
                self.__socket.connect((self.__ip, self.__port))
                break
            except Exception:
                continue
        self.get_logger().info("connected")

        while True:
            data = self.read_data()
            if len(data) == 0:
                continue
            self.publish_data(data)


def main():
    rclpy.init()
    udp = Udp()
    udp.run()


if __name__ == "__main__":
    main()
