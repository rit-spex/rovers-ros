#!/usr/bin/env python3
"""
File: Camera.py
Description: ROS 2 Node for capturing and publishing camera images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        # Initialize the node with a default name 'camera_node'
        # This name can be overwritten by launch files
        super().__init__('camera_node')

        # 1. Declare Parameters (ROS 2 requires declaring params before reading them)
        self.declare_parameter('name', 'rover_cam')
        self.declare_parameter('camera_index', 0)

        # 2. Read Parameters
        self.cam_name = self.get_parameter('name').get_parameter_value().string_value
        self.cam_index = self.get_parameter('camera_index').get_parameter_value().integer_value

        # 3. Create Publisher
        # Topic: cameras/<name>_topic
        topic_name = f'cameras/{self.cam_name}_topic'
        self.publisher_ = self.create_publisher(Image, topic_name, 10)

        # 4. Setup Camera Resource
        # Optimization: Open the camera ONCE here, not in the loop
        self.cap = cv2.VideoCapture(self.cam_index)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        # 3. Set Resolution explicitly (Optional but recommended)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Check if it opened
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video stream from index {self.cam_index}")
        else:
            self.get_logger().info(f"Initialized V4L2 camera on index {self.cam_index}")
        

        # 5. Create Timer (30 Hz)
        self.timer_period = 1.0 / 30
        self.timer = self.create_timer(self.timer_period, self.capture_callback)
        
        # 6. Initialize CV Bridge
        self.cv_bridge = CvBridge()

    def capture_callback(self):
        """Callback function called at 30Hz"""
        ret, frame = self.cap.read()

        if ret:
            # Convert to ROS message
            # Note: We use the header to stamp the time, which is important for sync
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self.cam_name
            
            self.publisher_.publish(image_msg)
        else:
            self.get_logger().warn(f"Failed to capture frame from {self.cam_name} camera.")

    def destroy_node(self):
        """Cleanup when node shuts down"""
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()