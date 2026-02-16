import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, UInt8
from cv_bridge import CvBridge
import time

class ArucoTracker(Node):
    def __init__(self):
        super().__init__("aruco_tracker")

        # --- 1. MODERN ARUCO SETUP (OpenCV 4.7+) ---
        self.marker_size = 0.1  # Meters

        # Define the dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Define parameters
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Create the Detector object (The new standard way)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # --- CAMERA CALIBRATION ---
        self.camera_matrix = np.array(
            [
                [1114.4638331, 0.00, 352.24603902],
                [0.00, 1040.32652878, 205.15271065],
                [0.00, 0.00, 1.00],
            ],
            dtype=np.float32,
        )

        self.dist_coeffs = np.array(
            [[-4.07243548e-01, 2.38916974e-01, 3.29860476e-03, -4.12199759e-03, -4.05068312e00]],
            dtype=np.float32,
        )

        # --- DEFINE 3D MARKER POINTS FOR POSE ESTIMATION ---
        # Top-left, Top-right, Bottom-right, Bottom-left
        half_size = self.marker_size / 2.0
        self.marker_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # --- LOGIC VARIABLES (Moved from callback to here) ---
        self.avg_dist_buffer = []
        self.num_frames = 3
        self.update_interval = 1.35
        self.last_update_time = time.time()
        self.display_distance = 0.00
        self.control_mode = 0

        # --- ROS SETUP ---
        self.bridge = CvBridge()
        self.frame_location = Point()
        self.pointFound = Bool()

        # Input
        self.image_sub = self.create_subscription(
            Image, "/cameras/rover_cam_topic", self.image_callback, 10
        )

        self.create_subscription(
            UInt8, "/BASESTATION/XBOX/CONTROL_MODE", self.control_mode_callback, 10
        )
        
        # Outputs
        # Published debug image for RQT
        self.image_pub = self.create_publisher(Image, "/object_detection/tracking_out", 10)
        self.point_pub = self.create_publisher(Point, "/object_detection/aruco_point_out", 10)
        self.id_pub = self.create_publisher(Bool, "/object_detection/aruco_id_out", 10)

        self.get_logger().info("ArUco Tracker Initialized (OpenCV 4+)")

    def image_callback(self, msg):
        if not (self.control_mode == 2):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. Detect Markers (Using the Detector class)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        isFound = False
        
        # Initialize display vars for this frame
        current_x = 0.0
        current_z = 0.0

        if ids is not None and len(ids) > 0:
            isFound = True
            
            # Draw marker outlines
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Loop through detected markers
            for i in range(len(ids)):
                # 2. Pose Estimation (Using solvePnP instead of deprecated estimatePoseSingleMarkers)
                success, rvec, tvec = cv2.solvePnP(
                    self.marker_points, 
                    corners[i], 
                    self.camera_matrix, 
                    self.dist_coeffs, 
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    # 3. Draw Axis (Modern Replacement for drawAxis)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

                    # Update location data (using the first marker found as primary)
                    if i == 0:
                        current_x = tvec[0][0]
                        y = tvec[1][0]
                        current_z = tvec[2][0]

                        self.frame_location.x = float(current_x)
                        self.frame_location.y = float(y)
                        self.frame_location.z = float(current_z)

                        # Distance Calculation
                        distance = np.linalg.norm(tvec)
                        self.avg_dist_buffer.append(distance)

            # --- SMOOTHING LOGIC ---
            # Calculate average if buffer is full
            if len(self.avg_dist_buffer) >= self.num_frames:
                new_avg = sum(self.avg_dist_buffer) / len(self.avg_dist_buffer)
                self.avg_dist_buffer = []  # Clear buffer
                
                # Only update the text display every X seconds
                if time.time() - self.last_update_time >= self.update_interval:
                    self.display_distance = new_avg
                    self.last_update_time = time.time()
            
            # --- TEXT UI ---
            cv2.putText(frame, f"ID: {ids[0][0]}", (250, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"X: {current_x:.2f}m", (250, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Z: {current_z:.2f}m", (250, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        else:
            # Marker Lost
            isFound = False
            self.avg_dist_buffer = [] # Reset buffer on loss
            cv2.putText(frame, "Lost Tracking", (250, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Always display the last known stable distance
        cv2.putText(frame, f"Dist: {self.display_distance:.2f}m", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # --- PUBLISH OUTPUTS ---
        # 1. Publish Data
        self.pointFound.data = isFound
        self.point_pub.publish(self.frame_location)
        self.id_pub.publish(self.pointFound)

        # 2. Publish Debug Image (For RQT)
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(out_msg)
        except Exception as e:
            pass

    def control_mode_callback(self, msg: UInt8):
        self.control_mode = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()