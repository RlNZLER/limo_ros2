import cv2
import numpy as np
from darknet import darknet
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')
        self.bridge = CvBridge()

        # YOLO configuration
        self.net = darknet.load_net(b"cfg/yolov3.cfg", b"yolov3.weights", 0)
        self.meta = darknet.load_meta(b"cfg/coco.data")

        # ROS subscriptions and publications
        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.camera_info_callback, 
                                                qos_profile=qos_profile_sensor_data)
        self.object_location_pub = self.create_publisher(PoseStamped, '/limo/object_location', 10)
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', 
                                                  self.image_callback, qos_profile=qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_tf_transform(self, target_frame, source_frame):
        # Similar to the previous example
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def camera_info_callback(self, data):
        # Similar to the previous example
        pass

    def image_callback(self, data):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # YOLO object detection
        detections = darknet.detect_np(self.net, self.meta, cv_image)
        
        for detection in detections:
            label, confidence, bbox = detection
            x, y, w, h = bbox

            # Assuming the pink color range in HSV
            lower_pink = np.array([300, 80, 70])
            upper_pink = np.array([310, 100, 100])

            # Extract the region of interest (ROI) based on YOLO bounding box
            roi = cv_image[int(y):int(y+h), int(x):int(x+w)]

            # Convert BGR to HSV
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # Thresholding to detect pink color
            mask_pink = cv2.inRange(hsv_roi, lower_pink, upper_pink)

            # Check if pink color is present in the YOLO-detected object
            if cv2.countNonZero(mask_pink) > 0:
                # Coordinates in the image frame
                image_coords = ((x + w/2), (y + h/2))

                # Convert to world coordinates using TF transforms
                transform = self.get_tf_transform('depth_link', 'odom')
                p_camera = do_transform_pose(image_coords, transform)

                # Print the coordinates in the odom frame
                print('Odom coords:', p_camera.position)

                transform = self.get_tf_transform('depth_link', 'map')
                p_camera = do_transform_pose(image_coords, transform)

                # Print the coordinates in the map frame
                print('Map coords:', p_camera.position)

                # Publish the object location
                object_location = PoseStamped()
                object_location.header.frame_id = "depth_link"
                object_location.pose.orientation.w = 1.0
                object_location.pose.position.x = p_camera.position.x
                object_location.pose.position.y = p_camera.position.y
                object_location.pose.position.z = p_camera.position.z
                self.object_location_pub.publish(object_location)

def main(args=None):
    rclpy.init(args=args)
    yolo_object_detector = ObjectDetector()
    rclpy.spin(yolo_object_detector)
    yolo_object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
