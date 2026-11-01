import rclpy
from rclpy import Node


class Auger(Node):
    def __init__(self):
        super().__init__("auger_node")

    def run(self):
        self.get_logger().info("starting auger")
        rclpy.spin(self)
        self.get_logger().info("stopping auger")


def main():
    rclpy.init()
    auger = Auger()
    auger.run()


if __name__ == "__main__":
    main()
