from typing import Any
from constants.CAN_Constants import TOPICS
from constants.CommandCodes import CONSTANTS

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16, Int32
from Jetson.
from STorM32_lib import *

class CameraGimbal(Node):

    # Configuration
    baud : Int32 = 115200
    UART_port = "COM3" 

    # Camera Control Inputs
    yaw_deg : Float32
    pitch_deg : Float32
    roll_deg : Float32
    # Camera Control Limits
    yaw_max : Float32 = 45.0
    yaw_min : Float32 = -45.0
    pitch_max : Float32 = 90.0
    pitch_min : Float32 = -90.0
    roll_max : Float32 = 45.0
    roll_min : Float32 = -45.0

    def __init__(self):
        # Initialize ROS node
        super().__init__("camera_gimbal_node")

        # Start serial
        ser = serial.Serial(UART_port, baud)

        # Set initial angles
        reset_angles()

        # Subscribe to ESTOP
        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

    def reset_angles():
        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping status LED...")
        
        rclpy.shutdown()

    def run(self):
        self.get_logger().info("starting status LED...")
        rclpy.spin(self)
        self.get_logger().info("stopping status LED")


def main():
    rclpy.init()
    camera_gimbal = CameraGimbal()
    camera_gimbal.run()


if __name__ == "__main__":
    main()
