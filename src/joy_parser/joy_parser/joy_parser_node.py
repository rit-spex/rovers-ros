import rclpy
from rclpy.node import Node


class JoyParser(Node):
    def __init__(self) -> None:
        super().__init__("joy_parser")

    def run(self) -> None:
        rclpy.spin(self)


def main():
    rclpy.init()
    joy_parser = JoyParser()
    joy_parser.run()


if __name__ == "__main__":
    main()
