import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import UInt8MultiArray
from digi.xbee.devices import XBeeDevice, TimeoutException

# from custom_interfaces.msg import Can
# from constants.constants.CAN_Constants import CHANNEL, TOPICS


class Xbee(Node):
    __port: str
    __baud_rate: int
    __xbee_device: XBeeDevice
    __publisher: Publisher

    def __init__(self) -> None:
        super().__init__("xbee_node")

        self.__port = "/dev/ttyUSB0"
        self.__baud_rate = 230400
        self.__xbee_device = XBeeDevice(self.__port, self.__baud_rate)
        self.__publisher = self.create_publisher(
            UInt8MultiArray, "BASESTATION/MESSAGES", 10
        )

    def read_data(self) -> list[int] | None:
        message = None
        try:
            message = self.__xbee_device.read_data(0.0004)
            if message is None:
                return None
            return list(message.data)
        except TimeoutException:
            return []
            # self.get_logger().info("timed out")
        except Exception:
            self.get_logger().info("failed to read data")
            return None

    def publish_data(self, data: list[int]) -> None:
        msg = UInt8MultiArray()
        msg.data = data
        self.__publisher.publish(msg)

    def run(self) -> None:
        self.get_logger().info("starting xbee...")
        self.__xbee_device.open()

        while True:
            data = self.read_data()
            if data is None:
                self.get_logger().info("message was none")
                break
            if len(data) == 0:
                continue
            self.publish_data(data)


def main() -> None:
    rclpy.init()
    xbee = Xbee()
    xbee.run()


if __name__ == "__main__":
    main()
