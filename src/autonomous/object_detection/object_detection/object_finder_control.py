import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
import math
import time


class ObjectSearchAndFollow(Node):
    def __init__(self):
        super().__init__("object_finder_control")

        # --- CONFIGURATION ---
        self.IMAGE_WIDTH = 640
        self.TARGET_AREA = 40000.0
        
        # Search Parameters
        self.SEARCH_SPEED = 0.2      # Linear speed while searching (0 for rotate-in-place)
        self.SEARCH_AMPLITUDE = 0.0 # Radians (~45 degrees) to sweep left/right
        self.SEARCH_FREQ = 0.5       # How fast to sweep (Hz)
        
        # PID Gains
        self.KP_VISUAL = 0.002       # Camera turning gain
        self.KP_IMU = 2.0            # IMU turning gain
        self.MAX_ANG_VEL = 0.5
        self.MAX_LIN_VEL = 0.5
        self.AREA_SCALE = math.sqrt(0.0001)
        self.KI_VISUAL = 0.0

        # State Variables
        self.last_detection_time = 0.0
        self.detection_timeout = 0.5 # Seconds to wait before switching back to search
        self.current_yaw = 0.0
        self.start_yaw = None        # Will store the yaw when we first start up
        self.object_visible = False
        self.bbox_center = 0.0
        self.bbox_area = 0.0

        # --- TOPICS ---
        self.IMU_TOPIC = "/imu/data" 
        self.DETECTION_TOPIC = "/object_detection/bbox"
        
        # --- SUBSCRIBERS ---
        self.imuSub = self.create_subscription(
            Imu, self.IMU_TOPIC, self.imu_callback, 10
        )
        self.detSub = self.create_subscription(
            Float32MultiArray, self.DETECTION_TOPIC, self.detection_callback, 10
        )

        # --- PUBLISHERS ---
        self.velPubRight = self.create_publisher(Float32, "/object_detection/OD_RY", 10)
        self.velPubLeft = self.create_publisher(Float32, "/object_detection/OD_LY", 10)

        # --- MAIN CONTROL LOOP ---
        # Run at 20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def imu_callback(self, msg):
        # Convert Quaternion to Euler (Yaw only)
        q = msg.orientation
        # Manual conversion to avoid dependency on tf_transformations
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Set the "zero" heading when we first get data
        if self.start_yaw is None:
            self.start_yaw = self.current_yaw

    def detection_callback(self, msg):
        # Expecting [x1, y1, x2, y2, class_id]
        if len(msg.data) >= 4:
            x1, y1, x2, y2 = msg.data[0:4]
            width = x2 - x1
            height = y2 - y1
            
            self.bbox_area = width * height
            self.bbox_center = (x1 + x2) / 2.0
            
            self.object_visible = True
            self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
        else:
            # Empty array or bad data usually means no detection
            self.object_visible = False

    def control_loop(self):
        # 1. Determine State
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_detection = current_time - self.last_detection_time

        linear_out = 0.0
        angular_out = 0.0

        # --- STATE: TRACKING (Object Found) ---
        if self.object_visible and time_since_detection < self.detection_timeout:
            
            # Stop if too close
            if self.bbox_area > self.TARGET_AREA:
                self.stop_robot()
                return

            # Visual Error: Center of Image vs Center of BBox
            error_pixels = (self.IMAGE_WIDTH / 2.0) - self.bbox_center
            
            # Visual PID
            angular_out = self.KP_VISUAL * error_pixels * (math.sqrt(self.bbox_area)*self.AREA_SCALE)
            #self.get_logger().info(f"Angular Out: {angular_out}...")

            linear_out = self.SEARCH_SPEED # Move forward while tracking

        # --- STATE: SEARCHING (IMU Sweep) ---
        else:
            if self.start_yaw is None:
                return # Wait for IMU to initialize

            # Generate Search Path (Sine Wave relative to start heading)
            # Yaw Target = Start + Amplitude * sin(freq * time)
            t = current_time - self.start_time
            yaw_offset = self.SEARCH_AMPLITUDE * math.sin(2 * math.pi * self.SEARCH_FREQ * t)
            target_yaw = self.start_yaw + yaw_offset

            # Calculate Error (Shortest path between angles)
            error_yaw = target_yaw - self.current_yaw
            # Normalize angle to [-pi, pi] to prevent spinning the wrong way
            while error_yaw > math.pi: error_yaw -= 2 * math.pi
            while error_yaw < -math.pi: error_yaw += 2 * math.pi

            # IMU PID
            angular_out = self.KP_IMU * error_yaw
            linear_out = self.SEARCH_SPEED

        # 2. Clamp and Drive
        angular_out = max(min(angular_out, self.MAX_ANG_VEL), -self.MAX_ANG_VEL)
        
        # Differential Drive Mixer
        left_vel = linear_out - 0.4*angular_out
        right_vel = linear_out + 0.4*angular_out

        # Final Clamp for Motor Driver (-1.0 to 1.0)
        left_vel = max(min(left_vel, self.MAX_LIN_VEL), -self.MAX_LIN_VEL)
        right_vel = max(min(right_vel, self.MAX_LIN_VEL), -self.MAX_LIN_VEL)

        self.publish_velocity(left_vel, right_vel)

    def publish_velocity(self, left, right):
        l_msg = Float32()
        r_msg = Float32()
        l_msg.data = left  # Inverted per your original script
        r_msg.data = right 
        self.velPubLeft.publish(l_msg)
        self.velPubRight.publish(r_msg)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSearchAndFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()