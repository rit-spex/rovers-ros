# DEPENDENCIES:
# sudo apt install ros-humble-vision-opencv
# pip install opencv-contrib-python numpy


import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class ArucoTracker(Node):
    def __init__(self):
        super().__init__("aruco_tracker")

        # Load ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Camera Calibration
        # data = np.load("PATH")  # Update path
        # self.camera_matrix = data["camera_matrix"]
        # self.dist_coeffs = data["dist_coeffs"]

        # Camera calibration parameters
        self.camera_matrix = np.array(
            [
                [1.166475980e03, 0, 2.95126234e02],
                [0, 1.35364677e03, 2.7632291215e02],
                [0, 0, 1],
            ]
        )  # Replace with actual values
        self.dist_coeffs = np.zeros(
            (5, 1)
        )  # If calibrated, this is for distortion values

        # ArUco marker size (in meters)
        self.marker_size = 0.1

        # ROS2 Subscribers & Publishers
        self.bridge = CvBridge()
        self.frame_location = Point()
        self.pointFound = Bool()
        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )
        self.image_pub = self.create_publisher(Image, "/tracking/image_out", 10)
        self.point_pub = self.create_publisher(Point, "/tracking/point_out", 10)
        self.id_pub = self.create_publisher(Bool, "/tracking/id_out", 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        # Rotate
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        if ids is not None:
            isFound = True
            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Estimate Pose
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                for j in range(len(ids)):
                    x = tvec[i][0][0]
                    y = tvec[i][0][1]
                    z = tvec[i][0][2]
                    self.frame_location.x = y
                    self.frame_location.y = x
                    self.frame_location.z = z
                # print(f"Maker ID: {ids[i][0]}")
                # print(f"X: {x: .2f}, Y: {y: .2f}, Z: {z: .2f}")
                cv2.aruco.drawAxis(
                    frame, self.camera_matrix, self.dist_coeffs, rvec[i], tvec[i], 0.05
                )

                # Distance to marker
            distance = np.linalg.norm(tvec[i])

            # Frame Disp
            rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            cv2.putText(
                rotated_frame,
                f"Distance: {distance:.2f}m",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )  # Display UI

            cv2.putText(
                rotated_frame,
                f"X: {y:.2f}m",
                (250, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )  # Display UI

            cv2.putText(
                rotated_frame,
                f"Z: {z:.2f}m",
                (250, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )  # Display UI

            # Display Tracking Status
            cv2.putText(
                rotated_frame,
                "Tracking ArUco ID: {}".format(ids[0][0]),
                (250, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        else:
            isFound = False
            cv2.putText(
                rotated_frame,
                f"Lost Tracking",
                (250, 125),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        cv2.imshow("ArUco Tracking", rotated_frame)
        rotated_frame_image = self.bridge.cv2_to_imgmsg(rotated_frame, encoding="bgr8")
        self.image_pub.publish(rotated_frame_image)
        self.pointFound.data = isFound

        self.point_pub.publish(self.frame_location)
        self.id_pub.publish(self.pointFound)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
