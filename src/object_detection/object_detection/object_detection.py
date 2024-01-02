# Python libs
import rclpy
from rclpy.node import Node
from rclpy import qos

# OpenCV
import cv2

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener

# ROS Messages
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

# Object detection using YOLOv8n
from ultralytics import YOLO


class ObjectDetector(Node):
    pothole_counter = 0
    detected_potholes = []
    camera_model = None
    image_depth_ros = None
    waypoint_follower_status = True

    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    color2depth_aspect = 1.0 # for a simulated camera

    def __init__(self):    
        super().__init__('object_detection')
        self.bridge = CvBridge()

        self.waypoint_follower_status_sub = self.create_subscription(String, '/waypoint_follower/status', self.waypoint_follower_status_callback, 10)

        if self.waypoint_follower_status:
            self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                    self.camera_info_callback, 
                                                    qos_profile=qos.qos_profile_sensor_data)
            
            self.object_detection_status_pub = self.create_publisher(String, '/object_detection/status', 10)
            self.object_location_pub = self.create_publisher(PoseStamped, '/object_detection/location', 10)

            self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', 
                                                    self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
            
            self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', 
                                                    self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
            
            self.image_with_boxes_pub = self.create_publisher(Image, '/object_detection/yolo_detected_image', 10)
        
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

    def waypoint_follower_status_callback(self, msg):
        self.get_logger().info(f"Received waypoint follower status: {msg.data}")
        if msg.data == "Reached my origin!":
            self.waypoint_follower_status = True
            self.get_logger().info("Waypoint follower status set to True.")
                   
    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.object_detection_status_pub.publish(msg)

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None
        
    def calculate_pothole_info(self, x_min, y_min, x_max, y_max, depth_image, image_color):
        # Calculate centroid of the bounding box
        centroid_x = (x_min + x_max) / 2
        centroid_y = (y_min + y_max) / 2

        # Calculate depth coordinates at the centroid
        depth_coords = (
            depth_image.shape[0] / 2 + (centroid_y - image_color.shape[0] / 2) * self.color2depth_aspect,
            depth_image.shape[1] / 2 + (centroid_x - image_color.shape[1] / 2) * self.color2depth_aspect,
        )

        # Get the depth reading at the centroid location
        depth_value = depth_image[int(depth_coords[0]), int(depth_coords[1])]

        # Calculate object's 3D location in camera coordinates
        camera_coords = self.camera_model.projectPixelTo3dRay((centroid_x, centroid_y))
        camera_coords = [x / camera_coords[2] for x in camera_coords]
        camera_coords = [x * depth_value for x in camera_coords]
        
        object_location = PoseStamped()
        object_location.header.frame_id = "depth_link"
        object_location.pose.orientation.w = 1.0
        object_location.pose.position.x = camera_coords[0]
        object_location.pose.position.y = camera_coords[1]
        object_location.pose.position.z = camera_coords[2]     

        # print out the coordinates in the odom frame
        transform = self.get_tf_transform('depth_link', 'odom')
        p_camera = do_transform_pose(object_location.pose, transform)
        
        odom_coords = p_camera.position
        
        # publish so we can see that in rviz
        self.object_location_pub.publish(object_location)   
        
        return {
            "centroid_x": centroid_x,
            "centroid_y": centroid_y,
            "depth_value": depth_value,
            "odom_coords": odom_coords,
            "box_size": (x_max - x_min) * (y_max - y_min),
        }
        
    def process_detected_potholes(self, results, image_depth, image_color):
        for pothole_num, result in enumerate(results):
            for pred in result.boxes.xyxy:
                x_min, y_min, x_max, y_max = map(int, pred[:4])
                # Draw bounding box on the image
                cv2.rectangle(image_color, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # # Display the image with bounding boxes
                # cv2.imshow("YOLO Object Detection", image_color)
                # cv2.waitKey(1)
                
                image_with_boxes_msg = self.bridge.cv2_to_imgmsg(image_color, encoding="bgr8")
                self.image_with_boxes_pub.publish(image_with_boxes_msg)
                
                # Calculate pothole information
                pothole_info = self.calculate_pothole_info(x_min, y_min, x_max, y_max, image_depth, image_color)
                self.detected_potholes.append(pothole_info)

                # Print pothole information
                print(f"Pothole {self.pothole_counter + 1} - Odom Coordinates: {pothole_info['odom_coords']}, Size: {pothole_info['box_size']}")

                # Increment the pothole counter
                self.pothole_counter += 1
                
    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        self.publish_status("Object detection started!")
        # Perform YOLO object detection
        model = YOLO("src/object_detection/object_detection/runs/detect/train2/weights/best.pt")
        results = model.predict(source=image_color, save_txt=True, save=False, stream=True)
        
        # Process and print pothole information
        self.process_detected_potholes(results, image_depth, image_color)

def main(args=None):
    # --- Init
    rclpy.init(args=args)
    object_detection = ObjectDetector()
    
    rclpy.spin(object_detection)
    object_detection.publish_status("Object detection stopped!")
    
    object_detection.destroy_node()
    
    # --- Shut down
    rclpy.shutdown()

if __name__ == '__main__':
    main()