import rclpy
from rclpy import Node


class Carousel(Node):
    def __init__(self):
        super().__init__("carousel_node")

    def run(self):
        self.get_logger().info("starting carousel")
        rclpy.spin(self)
        self.get_logger().info("stopping carousel")


def main():
    rclpy.init()
    carousel = Carousel()
    carousel.run()


if __name__ == "__main__":
    main()
