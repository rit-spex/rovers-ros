#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import math
from rclpy.qos import qos_profile_sensor_data


# Paste into terminal and change lat and longitude to set target
"""
ros2 topic pub --once /GPS/decimal_target sensor_msgs/msg/NavSatFix "{header: {frame_id: 'map'}, latitude: 43.0845, longitude: -77.6743, altitude: 153.0}"

Use this for lidar cloud
ros2 run tf2_ros static_transform_publisher 0 0 0 -0.4 0 0 map unilidar_lidar
"""


class GPSIMUPathfinding(Node):
    def __init__(self):
        super().__init__("gps_imu_pathfinding")

        # --- Parameters ---
        self.declare_parameter("speed", 0.3)
        self.declare_parameter("turn_gain", 2.0)
        self.declare_parameter("stop_distance", 2.0)

        # Obstacle Avoidance Parameters
        self.declare_parameter("safe_distance", 1.5)  # Distance to trigger avoidance
        self.declare_parameter("obs_turn_gain", 0.6)  # Steering strength for dodging

        self.base_speed = self.get_parameter("speed").value
        self.turn_gain = self.get_parameter("turn_gain").value
        self.stop_dist = self.get_parameter("stop_distance").value
        self.safe_dist = self.get_parameter("safe_distance").value
        self.obs_turn_gain = self.get_parameter("obs_turn_gain").value
        self.imu_offset = 0

        # --- State ---
        self.current_lat = None
        self.current_lon = None
        self.target_lat = None
        self.target_lon = None
        self.current_heading = 0.0

        # Obstacle State
        self.obstacle_override = False
        self.obs_turn_adjust = 0.0

        # --- Subscribers ---
        self.create_subscription(NavSatFix, "/GPS/ROVER", self.gps_callback, 10)
        self.create_subscription(Imu, "/unilidar/imu", self.imu_callback, 10)
        self.create_subscription(
            NavSatFix, "/GPS/decimal_target", self.decimal_target_callback, 10
        )
        self.create_subscription(
            PointStamped, "/GPS/map_target", self.map_target_callback, 10
        )

        # Lidar Subscriber (Assuming pointcloud_to_laserscan outputs to /scan)
        self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data
        )

        # --- Publishers ---
        self.pub_left = self.create_publisher(Float32, "/object_detection/GPS_LY", 10)
        self.pub_right = self.create_publisher(Float32, "/object_detection/GPS_RY", 10)

        # --- Loop ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("GPS + IMU + Obstacle Avoidance Node Started.")

    def lidar_callback(self, msg):
        """
        Reads the 2D LaserScan and determines if obstacles are too close in the
        Front, Left, or Right sectors.
        """
        ranges = msg.ranges
        num_rays = len(ranges)
        if num_rays == 0:
            return

        # Helper function to find a specific angle's index in the ranges array
        def angle_to_index(angle_rad):
            idx = int((angle_rad - msg.angle_min) / msg.angle_increment)
            # Clamp index to array bounds
            return max(0, min(idx, num_rays - 1))

        # Helper function to safely get the minimum distance in a slice
        def get_min_dist(slice_ranges):
            valid_ranges = [
                r
                for r in slice_ranges
                if not math.isinf(r)
                and not math.isnan(r)
                and msg.range_min < r < msg.range_max
            ]
            return min(valid_ranges) if valid_ranges else float("inf")

        # Define sectors (Front 90 degrees total: -45 to +45)
        idx_right_outer = angle_to_index(math.radians(-45))
        idx_right_inner = angle_to_index(math.radians(-15))
        idx_left_inner = angle_to_index(math.radians(15))
        idx_left_outer = angle_to_index(math.radians(45))

        # Handle potential backward indexing depending on lidar rotation
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

        # --- Avoidance Logic ---
        # Note: In your current kinematic setup, positive turn_adjust increases Left Wheel
        # and decreases Right Wheel -> Robot turns RIGHT.
        if (
            min_front < self.safe_dist
            or min_left < self.safe_dist
            or min_right < self.safe_dist
        ):
            self.obstacle_override = True

            if min_front < self.safe_dist:
                # Obstacle dead ahead. Turn toward the side with more space.
                if min_left > min_right:
                    self.obs_turn_adjust = -self.obs_turn_gain  # Turn Left
                else:
                    self.obs_turn_adjust = self.obs_turn_gain  # Turn Right
            elif min_left < self.safe_dist:
                self.obs_turn_adjust = (
                    self.obs_turn_gain
                )  # Dodging left object -> Turn Right
            elif min_right < self.safe_dist:
                self.obs_turn_adjust = (
                    -self.obs_turn_gain
                )  # Dodging right object -> Turn Left

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)

        self.current_heading = yaw_deg + self.imu_offset
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

        distance = self.haversine_distance(
            self.current_lat, self.current_lon, self.target_lat, self.target_lon
        )
        target_bearing = self.calculate_bearing(
            self.current_lat, self.current_lon, self.target_lat, self.target_lon
        )

        if distance < self.stop_dist:
            self.publish_velocities(0.0, 0.0)
            return

        # Choose between Obstacle Avoidance and GPS Tracking
        if self.obstacle_override:
            turn_adjust = self.obs_turn_adjust
            current_speed = self.base_speed * 0.5  # Slow down by 50% while dodging
            self.get_logger().debug("DODGING OBSTACLE")
        else:
            heading_error = target_bearing - self.current_heading
            heading_error = (heading_error + 180) % 360 - 180
            turn_adjust = heading_error * (self.turn_gain / 100.0)
            current_speed = self.base_speed

        turn_adjust = max(min(turn_adjust, 1.0), -1.0)

        left = current_speed + turn_adjust
        right = current_speed - turn_adjust

        left = max(min(left, 1.0), -1.0)
        right = max(min(right, 1.0), -1.0)

        self.publish_velocities(left, right)

    def publish_velocities(self, l, r):
        self.pub_left.publish(Float32(data=l))
        self.pub_right.publish(Float32(data=r))

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
