import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics.models import YOLO

class YOLOTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        # --- PARAMETERS ---
        # Replace 'yolo26n.pt' with the absolute path to your model if needed
        self.declare_parameter('model_path', '/home/spex-rover/SPEX/rovers-ros/src/autonomous/object_detection/object_detection/last.pt') 
        self.declare_parameter('camera_topic', '/cameras/rover_cam_topic')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('target_class_id', -1) # -1 means track ALL classes

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.target_class = self.get_parameter('target_class_id').get_parameter_value().integer_value

        # --- LOAD MODEL ---
        self.get_logger().info(f"Loading YOLO model from: {model_path}...")
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise e

        # --- ROS SETUP ---
        self.bridge = CvBridge()
        
        # Subscriber: Camera Image
        self.sub_image = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        # Publisher: Data for Control Node
        # Format: [x1, y1, x2, y2, confidence, class_id, track_id]
        self.pub_bbox = self.create_publisher(Float32MultiArray, '/object_detection/bbox', 10)

        # Publisher: Debug Image (Annotated)
        self.pub_debug = self.create_publisher(Image, '/object_detection/debug_image', 10)

        self.get_logger().info("YOLO Tracker Node Initialized.")

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 2. Run Inference (Tracking Mode)
        # persist=True enables the internal tracker (ByteTrack/BoT-SORT) to keep IDs consistent
        results = self.model.track(
            cv_image, 
            persist=True, 
            conf=self.conf_thres, 
            verbose=False
        )
        
        # 3. Process Detections
        # We need to pick ONE primary target to send to the control node.
        # Strategy: Pick the largest bounding box (closest object) that matches our target class.
        
        best_box = None
        max_area = 0.0

        if results[0].boxes:
            for box in results[0].boxes:
                # box.xyxy is [[x1, y1, x2, y2]]
                coords = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                
                # Check Track ID (if available, sometimes it's None in first frame)
                track_id = int(box.id[0].item()) if box.id is not None else -1

                # Filter by Class ID (if specified)
                if self.target_class != -1 and cls_id != self.target_class:
                    continue

                # Calculate Area
                width = coords[2] - coords[0]
                height = coords[3] - coords[1]
                area = width * height

                # Select best target (Largest Area)
                if area > max_area:
                    max_area = area
                    best_box = [
                        float(coords[0]), # x1
                        float(coords[1]), # y1
                        float(coords[2]), # x2
                        float(coords[3]), # y2
                        conf,
                        float(cls_id),
                        float(track_id)
                    ]

        # 4. Publish Detection
        if best_box:
            msg_out = Float32MultiArray()
            msg_out.data = best_box
            self.pub_bbox.publish(msg_out)
        else:
            # Optional: Publish empty array to indicate "Target Lost"
            self.pub_bbox.publish(Float32MultiArray(data=[]))

        # 5. Publish Debug Image
        # Plot the results on the frame
        annotated_frame = results[0].plot()
        debug_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.pub_debug.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_tracker = YOLOTracker()
    
    try:
        rclpy.spin(yolo_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()