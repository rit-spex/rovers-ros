from rclpy import Node
from rclpy.publisher import Publisher

from std_msgs.msg import ByteMultiArray

class Socket(Node):
    __publisher: Publisher

    def __init__(self) -> None:
        super().__init__("socket")

        self.__publisher = self.create_publisher(ByteMultiArray, "/SOCKET/MESSAGES", 10)

        self.run()

    def run(self) -> None:
        pass
