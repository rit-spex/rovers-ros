import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, UInt8
from sensor_msgs.msg import Imu, LaserScan
from rclpy.qos import qos_profile_sensor_data
import math
import time


class ObjectSearchAndFollow(Node):
    def __init__(self):
        super().__init__("object_finder_control")

        # --- PARAMETERS ---
        self.declare_parameter("enable_obstacle_avoidance", True)
        self.declare_parameter("safe_distance", 1.5)
        self.declare_parameter("obs_turn_gain", 0.5)

        # --- CONFIGURATION ---
        self.IMAGE_WIDTH = 640
        self.TARGET_AREA = 40000.0 / 4.0

        # Search Parameters
        self.SEARCH_SPEED = 0.25  # Linear speed while searching (0 for rotate-in-place)
        self.SEARCH_AMPLITUDE = 0.1  # Radians (~45 degrees) to sweep left/right
        self.SEARCH_FREQ = 0.1 / 2.0  # How fast to sweep (Hz)

        # PID Gains
        self.KP_VISUAL = 0.002  # Camera turning gain
        self.KP_IMU = 2.0  # IMU turning gain
        self.MAX_ANG_VEL = 0.5
        self.MAX_LIN_VEL = 0.5
        self.AREA_SCALE = math.sqrt(0.0001)
        self.KI_VISUAL = 0.0

        # State Variables
        self.last_detection_time = 0.0
        self.detection_timeout = 1.0  # Seconds to wait before switching back to search
        self.current_yaw = 0.0
        self.start_yaw = None  # Will store the yaw when we first start up
        self.object_visible = False
        self.bbox_center = 0.0
        self.bbox_area = 0.0

        # Obstacle State
        self.obstacle_override = False
        self.obs_turn_adjust = 0.0

        # Control Mode
        self.control_mode = 0

        # --- TOPICS ---
        self.IMU_TOPIC = "/unilidar/imu"
        self.DETECTION_TOPIC = "/object_detection/bbox"

        # --- SUBSCRIBERS ---
        self.imuSub = self.create_subscription(
            Imu, self.IMU_TOPIC, self.imu_callback, 10
        )
        self.detSub = self.create_subscription(
            Float32MultiArray, self.DETECTION_TOPIC, self.detection_callback, 10
        )
        self.create_subscription(
            UInt8, "/BASESTATION/XBOX/CONTROL_MODE1", self.control_mode_callback, 10
        )

        # Lidar Subscriber using Sensor Data QoS
        self.scanSub = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data
        )

        # --- PUBLISHERS ---
        self.velPubRight = self.create_publisher(Float32, "/object_detection/OD_RY", 10)
        self.velPubLeft = self.create_publisher(Float32, "/object_detection/OD_LY", 10)

        # --- MAIN CONTROL LOOP ---
        # Run at 20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def lidar_callback(self, msg):
        # 1. Check if avoidance is enabled via parameter
        if not (self.control_mode == 1):
            return

        if not self.get_parameter("enable_obstacle_avoidance").value:
            self.obstacle_override = False
            return

        ranges = msg.ranges
        num_rays = len(ranges)
        if num_rays == 0:
            return

        def angle_to_index(angle_rad):
            idx = int((angle_rad - msg.angle_min) / msg.angle_increment)
            return max(0, min(idx, num_rays - 1))

        def get_min_dist(slice_ranges):
            valid_ranges = [
                r
                for r in slice_ranges
                if not math.isinf(r)
                and not math.isnan(r)
                and msg.range_min < r < msg.range_max
            ]
            return min(valid_ranges) if valid_ranges else float("inf")

        idx_right_outer = angle_to_index(math.radians(-45))
        idx_right_inner = angle_to_index(math.radians(-15))
        idx_left_inner = angle_to_index(math.radians(15))
        idx_left_outer = angle_to_index(math.radians(45))

        if idx_right_outer > idx_left_outer:
            idx_right_outer, idx_left_outer = idx_left_outer, idx_right_outer
            idx_right_inner, idx_left_inner = idx_left_inner, idx_right_inner

        right_slice = ranges[idx_right_outer:idx_right_inner]
        front_slice = ranges[idx_right_inner:idx_left_inner]
        left_slice = ranges[idx_left_inner:idx_left_outer]

        min_right = get_min_dist(right_slice)
        min_front = get_min_dist(front_slice)
        min_left = get_min_dist(left_slice)

        self.obstacle_override = False
        self.obs_turn_adjust = 0.0
        safe_dist = self.get_parameter("safe_distance").value
        obs_gain = self.get_parameter("obs_turn_gain").value

        # --- Avoidance Logic ---
        # In this script's mixer: left = L - 0.4*A, right = L + 0.4*A
        # If A > 0 -> Left slower, Right faster -> Turns LEFT
        # If A < 0 -> Left faster, Right slower -> Turns RIGHT

        if min_front < safe_dist or min_left < safe_dist or min_right < safe_dist:
            self.obstacle_override = True

            if min_front < safe_dist:
                if min_left > min_right:
                    self.obs_turn_adjust = obs_gain  # Turn Left
                else:
                    self.obs_turn_adjust = -obs_gain  # Turn Right
            elif min_left < safe_dist:
                self.obs_turn_adjust = -obs_gain  # Dodge left object -> Turn Right
            elif min_right < safe_dist:
                self.obs_turn_adjust = obs_gain  # Dodge right object -> Turn Left

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.start_yaw is None:
            self.start_yaw = self.current_yaw

    def control_mode_callback(self, msg: UInt8):
        if msg.data == 1:
            self.start_yaw = None

        self.control_mode = msg.data

    def detection_callback(self, msg):
        if len(msg.data) >= 4:
            x1, y1, x2, y2 = msg.data[0:4]
            width = x2 - x1
            height = y2 - y1

            self.bbox_area = width * height
            self.bbox_center = (x1 + x2) / 2.0

            self.object_visible = True
            self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
        else:
            self.object_visible = False

    def control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_detection = current_time - self.last_detection_time

        linear_out = 0.0
        angular_out = 0.0

        # --- STATE 0: DODGING (Highest Priority) ---
        if self.obstacle_override:
            linear_out = self.SEARCH_SPEED * 0.7  # Slow down while dodging
            angular_out = self.obs_turn_adjust
            # self.get_logger().info("Dodging Obstacle!")

        # --- STATE 1: TRACKING (Object Found) ---
        elif self.object_visible and time_since_detection < self.detection_timeout:
            if self.bbox_area > self.TARGET_AREA:
                self.stop_robot()
                return

            error_pixels = (self.IMAGE_WIDTH / 2.0) - self.bbox_center
            angular_out = (
                self.KP_VISUAL
                * error_pixels
                * (math.sqrt(self.bbox_area) * self.AREA_SCALE)
            )
            linear_out = self.SEARCH_SPEED

        # --- STATE 2: SEARCHING (IMU Sweep) ---
        else:
            if self.start_yaw is None:
                return

            t = current_time - self.start_time
            yaw_offset = self.SEARCH_AMPLITUDE * math.sin(
                2 * math.pi * self.SEARCH_FREQ * t
            )
            target_yaw = self.start_yaw + yaw_offset

            error_yaw = target_yaw - self.current_yaw
            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi

            angular_out = self.KP_IMU * error_yaw
            linear_out = self.SEARCH_SPEED

        # 2. Clamp and Drive
        angular_out = max(min(angular_out, self.MAX_ANG_VEL), -self.MAX_ANG_VEL)

        # Differential Drive Mixer
        left_vel = linear_out - 0.4 * angular_out
        right_vel = linear_out + 0.4 * angular_out

        # Final Clamp for Motor Driver (-1.0 to 1.0)
        left_vel = max(min(left_vel, self.MAX_LIN_VEL), -self.MAX_LIN_VEL)
        right_vel = max(min(right_vel, self.MAX_LIN_VEL), -self.MAX_LIN_VEL)

        self.publish_velocity(left_vel, right_vel)

    def publish_velocity(self, left, right):
        l_msg = Float32()
        r_msg = Float32()
        l_msg.data = -left
        r_msg.data = -right
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
