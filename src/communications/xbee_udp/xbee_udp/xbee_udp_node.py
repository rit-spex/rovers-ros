# ros imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher

from std_msgs.msg import UInt8MultiArray

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

    def read_data(self, buffer_size: int) -> list[int]:
        try:
            data, addr = self.__socket.recvfrom(buffer_size)
            payload = list(data)
            self.get_logger().info(
                f"received {len(payload)} bytes from {addr[0]}:{addr[1]}: "
                f"{data.hex(' ')}"
            )

        except BlockingIOError:
            return []
        except OSError as e:
            # only error out if it wasn't from non-blocking
            if e.errno in (errno.EWOULDBLOCK, errno.EAGAIN):
                return []
            self.get_logger().error(f"failed to receive data: {e}")
            return []
        except Exception as e:
            self.get_logger().error(f"failed to receive data: {e}")
            return []
        return payload

    def publish_data(self, data: list[int]) -> None:
        msg = UInt8MultiArray()
        msg.data = data
        self.__publisher.publish(msg)

    def run(self) -> None:
        self.get_logger().info("starting xbee udp socket connection...")
        self.__socket.bind((self.__address, self.__port))

        # make it so it will not stop the code when requesting a read
        self.__socket.setblocking(False)

        try:
            while rclpy.ok():
                data = self.read_data(self.__buffer_size)
                if data:
                    self.publish_data(data)
                rclpy.spin_once(self, timeout_sec=0.01)
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
