from typing import Any
from constants.CAN_Constants import TOPICS
from constants.CommandCodes import CONSTANTS

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Bool, Float32, Float23MultiArray, UInt8MultiArray, Int8, Int16, Int32
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
        self.reset_angles()
        self.angles = cCMD_SETANGLES(ser, self.yaw_deg, self.pitch_deg, self.roll_deg)

        # Initialize STorM32 command object
        self.storm = cSTorM32RcCmd(ser=ser)

        self.timer=self.create_timer(0.1, self.timer_callback)

        # Subscribe to ESTOP
        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.__angle_publisher = self.create_publisher(
            msg_type=Float32MultiArray,
            topic="/camera_gimbal/angle",
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Float32MultiArray,
            topic="/camera_gimbal/angle",
            callback=self.__on_angles_received,
            qos_profile=10,
        )

    def timer_callback(self):
        # Update angles
        self.__angle_publisher.publish(Float32MultiArray([self.yaw_deg, self.pitch_deg, self.roll_deg]))

    def reset_angles(self):
        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0

    def update_angles(self, yaw: float, pitch: float, roll: float):
        self.yaw_deg = yaw
        self.pitch_deg = pitch
        self.roll_deg = roll
        self.angles = cCMD_SETANGLES(yaw, pitch, roll)
        self.angles.send()

    def __on_angles_received(self, msg: Float32MultiArray):
        # Handle received angles 
        self.update_angles(msg.data[0], msg.data[1], msg.data[2])
        self.get_logger().debug(f"Sent angles: Yaw: {self.yaw_deg}, Pitch: {self.pitch_deg}, Roll: {self.roll_deg}")

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
