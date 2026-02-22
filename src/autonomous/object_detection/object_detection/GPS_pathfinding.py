#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import math


class GPSIMUPathfinding(Node):
    def __init__(self):
        super().__init__("gps_imu_pathfinding")

        # --- Parameters ---
        self.declare_parameter("speed", 0.3)  # Base speed
        self.declare_parameter(
            "turn_gain", 2.0
        )  # Higher gain because IMU is responsive
        self.declare_parameter("stop_distance", 2.0)
        # self.declare_parameter('imu_offset', 0.0)    # DEGREES. Add this if 0 isn't North.

        self.base_speed = self.get_parameter("speed").value
        self.turn_gain = self.get_parameter("turn_gain").value
        self.stop_dist = self.get_parameter("stop_distance").value
        # self.imu_offset = self.get_parameter('imu_offset').value
        self.imu_offset = 0

        # --- State ---
        self.current_lat = None
        self.current_lon = None
        self.target_lat = None
        self.target_lon = None
        self.current_heading = 0.0  # -180 to 180 (0 = North, hopefully)

        # --- Subscribers ---
        # 1. GPS Position
        self.create_subscription(NavSatFix, "/GPS/ROVER", self.gps_callback, 10)

        # 2. IMU Heading (The new part!)
        # Note: Check if your topic is 'unilidar/imu' or 'unilidar/IMU'
        self.create_subscription(Imu, "/unilidar/imu", self.imu_callback, 10)

        # 3. Targets
        self.create_subscription(
            NavSatFix, "/GPS/decimal_target", self.decimal_target_callback, 10
        )
        self.create_subscription(
            PointStamped, "/GPS/map_target", self.map_target_callback, 10
        )

        # --- Publishers ---
        self.pub_left = self.create_publisher(Float32, "/object_detection/GPS_LY", 10)
        self.pub_right = self.create_publisher(Float32, "/object_detection/GPS_RY", 10)

        # --- Loop ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            "GPS + IMU Node Started. Point robot NORTH before starting!"
        )

    def imu_callback(self, msg):
        """
        Reads the quaternion from the Unitree Lidar IMU and converts to Yaw.
        """
        q = msg.orientation

        # Convert Quaternion (x,y,z,w) to Euler (Roll, Pitch, Yaw)
        # We manually do the math to avoid importing tf_transformations (which is often missing)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        # Convert to degrees (-180 to 180)
        yaw_deg = math.degrees(yaw_rad)

        # Apply Offset (if you didn't start facing North)
        # GPS standard: 0=North, 90=East.
        # ROS standard: 0=East, 90=North (CCW).
        # We usually need to swap this.
        # For now, let's assume the IMU outputs standard ROS yaw (0=East).
        # To make it match GPS (0=North), we might need to subtract 90.
        # TRY THIS FIRST:
        # If your robot spins in circles, change this line to: self.current_heading = -yaw_deg
        self.current_heading = yaw_deg + self.imu_offset

        # Normalize to -180 to 180
        self.current_heading = (self.current_heading + 180) % 360 - 180

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def decimal_target_callback(self, msg):
        self.target_lat = msg.latitude
        self.target_lon = msg.longitude
        self.get_logger().info(f"New Target: {self.target_lat}, {self.target_lon}")

    def map_target_callback(self, msg):
        self.target_lat = msg.point.x
        self.target_lon = msg.point.y

    def control_loop(self):
        if self.current_lat is None or self.target_lat is None:
            return

        # 1. Distance & Bearing to Target
        distance = self.haversine_distance(
            self.current_lat, self.current_lon, self.target_lat, self.target_lon
        )
        target_bearing = self.calculate_bearing(
            self.current_lat, self.current_lon, self.target_lat, self.target_lon
        )

        if distance < self.stop_dist:
            self.publish_velocities(0.0, 0.0)
            return

        # 2. Heading Error
        # GPS Bearing is usually 0=North, 90=East
        # IMU Yaw is usually 0=East, 90=North (if ROS standard)
        # We need to ensure these match.

        heading_error = target_bearing - self.current_heading

        # Normalize error (-180 to 180)
        # This prevents the robot from doing a 350-degree turn when a 10-degree turn would work
        heading_error = (heading_error + 180) % 360 - 180

        # 3. P-Controller
        turn_adjust = heading_error * (self.turn_gain / 100.0)  # Scaling factor
        turn_adjust = max(min(turn_adjust, 1.0), -1.0)

        left = self.base_speed + turn_adjust
        right = self.base_speed - turn_adjust

        # Clamp
        left = max(min(left, 1.0), -1.0)
        right = max(min(right, 1.0), -1.0)

        self.publish_velocities(left, right)

        # Debugging: Use this to calibrate your IMU Offset!
        # self.get_logger().info(f"Head: {self.current_heading:.1f} | Targ: {target_bearing:.1f} | Err: {heading_error:.1f}")

    def publish_velocities(self, l, r):
        self.pub_left.publish(Float32(data=l))
        self.pub_right.publish(Float32(data=r))

    # --- Math Helpers ---
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (
            math.sin(dphi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        )
        return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        # Calculates bearing where 0 = North, 90 = East
        y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
        x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(
            math.radians(lat1)
        ) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1))
        return (math.degrees(math.atan2(y, x)) + 360) % 360


def main(args=None):
    rclpy.init(args=args)
    node = GPSIMUPathfinding()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
