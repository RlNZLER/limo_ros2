"""
Object Detector Node

This ROS2 node performs object detection using the YOLOv8n model to identify potholes in a depth image obtained from a simulated depth camera. 
It subscribes to camera information, color image, and depth image topics, processes the images using YOLOv8n, and publishes information about detected 
potholes, including their location and size. Additionally, it publishes markers for visualization in Rviz.

Dependencies:
- ROS 2
- Python 3
- OpenCV
- YOLOv8n model (Ultralytics)

Subscribed Topics:
- /waypoint_follower/status (std_msgs/String): Receives status updates from the waypoint follower node.
- /waypoint_follower/status (std_msgs/String): Receives task completion updates from the waypoint follower node.
- /limo/depth_camera_link/camera_info (sensor_msgs/CameraInfo): Camera information for the depth camera.
- /limo/depth_camera_link/image_raw (sensor_msgs/Image): Color image from the simulated depth camera.
- /limo/depth_camera_link/depth/image_raw (sensor_msgs/Image): Depth image from the simulated depth camera.

Published Topics:
- /object_detection/status (std_msgs/String): Publishes status updates about the object detection process.
- /object_detection/location (geometry_msgs/PoseStamped): Publishes the 3D location of detected potholes in the 'depth_link' frame.
- /object_detection/yolo_detected_image (sensor_msgs/Image): Publishes the color image with bounding boxes drawn around detected potholes for visualization in Rviz.
- /object_detection/markers (visualization_msgs/Marker): Publishes markers for each detected pothole for visualization in Rviz.

Parameters:
- YOLOv8n_TRAINED_MODEL (str): Path to the trained YOLOv8n model.
- DETECTED_POTHOLE_FILE_PATH (str): Path to the file where detected pothole information is saved.

Author: Amel Varghese
Date: 11/01/2024
Github: https://github.com/RlNZLER/limo_ros2.git

References:
[1] Felipe (Computer vision engineer) 2023, 'Train Yolov8 object detection on a custom dataset | Step by step guide | Computer vision tutorial', YouTube, viewed 23 December, 2023,
https://youtu.be/m9fH9OWn8YM

[2] Antonio Brandi 2023, 'Self Driving and ROS 2 - Learn by Doing! Odometry & Control', Udemy, viewed 23 December, 2023,
https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/

[3] LCAS 2023, 'CMP9767_LIMO - Assignment Template', Github, viewed 23 December, 2023,
https://github.com/LCAS/CMP9767_LIMO/tree/main/assignment_template/assignment_template

[4] ros2-gbp 2023, 'find_object_2d-release', Github, viewed 23 December, 2023,
https://github.com/ros2-gbp/find_object_2d-release

[5] ultralytics 2023, 'YOLOv8', Github, viewed 23 December, 2023,
https://github.com/ultralytics/ultralytics?tab=AGPL-3.0-1-ov-file

[6] OpenAI 2022, 'ChatGPT: OpenAI's Conversational Language Model', OpenAI, viewed 11 January 2024,
https://chat.openai.com/
"""


# Python libs.
import rclpy
from rclpy import qos
from rclpy.node import Node

# OpenCV.
import cv2

# ROS libraries.
import image_geometry
from tf2_ros import Buffer, TransformListener

# ROS Messages.
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

# Object detection using YOLOv8n.
from ultralytics import YOLO

class ObjectDetector(Node):
    # Attributes for counting potholes and storing detected potholes.
    pothole_counter = 0
    pothole_counter = 0
    detected_potholes = []
    
    # Distance threshold for considering potholes.
    DISTANCE_THRESHOLD = 0.60
    camera_model = None
    image_depth_ros = None
    YOLOv8n_TRAINED_MODEL = "src/object_detection/object_detection/runs/detect/train3/weights/best.pt"
    DETECTED_POTHOLE_FILE_PATH = "src/object_detection/data/detected_potholes.txt"
    model = YOLO(YOLOv8n_TRAINED_MODEL)

    # Aspect ration between color and depth cameras.
    color2depth_aspect = 1.0 # for a simulated camera

    def __init__(self):
        # Initialize the object detector node.
        super().__init__('object_detection')
        
        # Initialize necessary components and flags.
        self.bridge = CvBridge()
        self.stop_detection_flag = False
        self.start_detection_flag = False
        
        # Set up subscriptions and publishers for relevant topics.
        self.waypoint_follower_status_sub = self.create_subscription(String, '/waypoint_follower/status', self.waypoint_follower_status_callback, 10)
        self.task_completed_sub = self.create_subscription(String, '/waypoint_follower/status', self.task_completed_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',self.camera_info_callback, qos_profile=qos.qos_profile_sensor_data)
        self.image_color_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
        self.image_depth_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
        self.object_detection_status_pub = self.create_publisher(String, '/object_detection/status', 10)
        self.object_location_pub = self.create_publisher(PoseStamped, '/object_detection/location', 10)
        self.image_with_boxes_pub = self.create_publisher(Image, '/object_detection/yolo_detected_image', 10)
        self.marker_pub = self.create_publisher(Marker, '/object_detection/markers', 10)
        
        # Set up transformation buffer and listener.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        
    # Callback for waypoint follower status.
    def waypoint_follower_status_callback(self, msg):
        if msg.data == "Reached my origin!":
            self.start_detection_flag = True
            self.publish_status("Starting object detection...")
            
    # Callback for task completion status.
    def task_completed_callback(self, msg):
        if msg.data == "Task Completed!":
            self.stop_detection_flag = True
            self.publish_status("Stopping object detection...")
    
    # Publishes the object detection status.
    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.object_detection_status_pub.publish(msg)
        self.get_logger().info(msg.data)

    # Gets the transformation between two frames.
    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None
    
    # Publishes a marker for a detected pothole on Rviz.
    def publish_marker(self, pothole_info):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "potholes"
        marker.id = self.pothole_counter
        marker.type = Marker.SPHERE  
        marker.action = Marker.ADD
        marker.pose.position.x = pothole_info['x_map']
        marker.pose.position.y = pothole_info['y_map']
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05 
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker on Rviz.
        self.marker_pub.publish(marker)
        
    # Calculates map coordinates from odom coordinates. 
    def calculate_map_coordinates(self, odom_coords):
        try:
            transform = self.get_tf_transform('map', 'odom')
            if transform:
                p_odom = Pose()
                p_odom.position.x = odom_coords.x
                p_odom.position.y = odom_coords.y
                p_odom.position.z = odom_coords.z
                p_map = do_transform_pose(p_odom, transform)
                return p_map.position.x, p_map.position.y
            else:
                self.get_logger().warning("Transform from 'odom' to 'map' not available.")
                return None, None
        except Exception as e:
            self.get_logger().warning(f"Error calculating map coordinates: {str(e)}")
            return None, None       
    
    # Calculates pothole size from bounding box coordinates and depth information.
    def calculate_pothole_size(self, x_min, x_max, y_min, y_max, depth_image, image_color):
        depth_coords_min = (
            depth_image.shape[0] / 2 + (y_min - image_color.shape[0] / 2) * self.color2depth_aspect,
            depth_image.shape[1] / 2 + (x_min - image_color.shape[1] / 2) * self.color2depth_aspect,
        )
        
        depth_coords_min_clamped = (
            min(depth_coords_min[0], 479),
            min(depth_coords_min[1], 479)
        )
        
        depth_value_min = depth_image[int(depth_coords_min_clamped[0]), int(depth_coords_min_clamped[1])]
            
        p_min = self.camera_model.projectPixelTo3dRay((x_min, y_min))
        p_min = [x / p_min[2] for x in p_min]
        p_min = [x * depth_value_min for x in p_min]
        
        p_min_odom_location = PoseStamped()
        p_min_odom_location.header.frame_id = "depth_link"
        p_min_odom_location.pose.orientation.w = 1.0
        p_min_odom_location.pose.position.x = p_min[0]
        p_min_odom_location.pose.position.y = p_min[1]
        p_min_odom_location.pose.position.z = p_min[2]  
        transform_d_to_o = self.get_tf_transform('odom', 'depth_link')
        p_min_camera = do_transform_pose(p_min_odom_location.pose, transform_d_to_o)
        p_min_odom_coords = p_min_camera.position
        
        transform_o_to_m = self.get_tf_transform('map', 'odom')
        p_min_odom = Pose()
        p_min_odom.position.x = p_min_odom_coords.x
        p_min_odom.position.y = p_min_odom_coords.y
        p_min_odom.position.z = p_min_odom_coords.z
        p_map = do_transform_pose(p_min_odom, transform_o_to_m)
        p_min_map_x = p_map.position.x
        p_min_map_y = p_map.position.y
        
        
        depth_coords_max = (
            depth_image.shape[0] / 2 + (y_max - image_color.shape[0] / 2) * self.color2depth_aspect,
            depth_image.shape[1] / 2 + (x_max - image_color.shape[1] / 2) * self.color2depth_aspect,
        )
        depth_coords_max_clamped = (
            min(depth_coords_max[0], 479),
            min(depth_coords_max[1], 479)
        )
        
        depth_value_max = depth_image[int(depth_coords_max_clamped[0]), int(depth_coords_max_clamped[1])]
        
        p_max = self.camera_model.projectPixelTo3dRay((x_max, y_max))
        p_max = [x / p_max[2] for x in p_max]
        p_max = [x * depth_value_max for x in p_max]
        
        p_max_odom_location = PoseStamped()
        p_max_odom_location.header.frame_id = "depth_link"
        p_max_odom_location.pose.orientation.w = 1.0
        p_max_odom_location.pose.position.x = p_max[0]
        p_max_odom_location.pose.position.y = p_max[1]
        p_max_odom_location.pose.position.z = p_max[2]  
        transform_d_to_o = self.get_tf_transform('odom', 'depth_link')
        p_max_camera = do_transform_pose(p_max_odom_location.pose, transform_d_to_o)
        p_max_odom_coords = p_max_camera.position
        
        transform_o_to_m = self.get_tf_transform('map', 'odom')
        p_max_odom = Pose()
        p_max_odom.position.x = p_max_odom_coords.x
        p_max_odom.position.y = p_max_odom_coords.y
        p_max_odom.position.z = p_max_odom_coords.z
        p_map = do_transform_pose(p_max_odom, transform_o_to_m)
        p_max_map_x = p_map.position.x
        p_max_map_y = p_map.position.y
        
        pothole_height = abs(p_max_map_y - p_min_map_y)
        pothole_width = abs(p_max_map_x - p_min_map_x)
        
        pothole_size = pothole_height*pothole_width
        
        return pothole_height, pothole_width, pothole_size

    # Calculates and publishes information for a detected pothole.    
    def calculate_pothole_info(self, x_min, y_min, x_max, y_max, depth_image, image_color):
        # Calculate centroid of the bounding box.
        centroid_x = (x_min + x_max) / 2
        centroid_y = (y_min + y_max) / 2

        # Calculate depth coordinates at the centroid.
        depth_coords = (
            depth_image.shape[0] / 2 + (centroid_y - image_color.shape[0] / 2) * self.color2depth_aspect,
            depth_image.shape[1] / 2 + (centroid_x - image_color.shape[1] / 2) * self.color2depth_aspect,
        )

        # Get the depth reading at the centroid location.
        depth_value = depth_image[int(depth_coords[0]), int(depth_coords[1])]
        
        # Calculate object's 3D location in camera coordinates.
        camera_coords = self.camera_model.projectPixelTo3dRay((centroid_x, centroid_y))
        camera_coords = [x / camera_coords[2] for x in camera_coords]
        camera_coords = [x * depth_value for x in camera_coords]
        
        object_location = PoseStamped()
        object_location.header.frame_id = "depth_link"
        object_location.pose.orientation.w = 1.0
        object_location.pose.position.x = camera_coords[0]
        object_location.pose.position.y = camera_coords[1]
        object_location.pose.position.z = camera_coords[2]     

        # Calculate odom coordinates from depth link frame.
        transform = self.get_tf_transform('odom', 'depth_link')
        p_camera = do_transform_pose(object_location.pose, transform)
        odom_coords = p_camera.position
        
        # Calculate map coordinates
        x_map, y_map = self.calculate_map_coordinates(odom_coords)
        
        # Calculate pothole height, width and size.
        height, width, size = self.calculate_pothole_size(x_min, y_min, x_max, y_max, depth_image, image_color)
        
        # Publish the object_location information on the topic.
        self.object_location_pub.publish(object_location)   
        
        return {
            "centroid_x": centroid_x,
            "centroid_y": centroid_y,
            "depth_value": depth_value,
            "odom_coords": odom_coords,
            "depth_value": depth_value,
            "x_map": x_map,
            "y_map": y_map,
            "height": height,
            "width": width,
            "box_size": size
        }
        
    # Processes detected potholes and updates relevant information.
    def process_detected_potholes(self, results, image_depth, image_color):
        if not self.start_detection_flag:
            return
        if not self.stop_detection_flag:
            self.publish_status("Object detection running!")
        for pothole_num, result in enumerate(results):
            if self.stop_detection_flag:
                break
            for pred in result.boxes.xyxy:
                x_min, y_min, x_max, y_max = map(int, pred[:4])
                
                # Calculate pothole information
                pothole_info = self.calculate_pothole_info(x_min, y_min, x_max, y_max, image_depth, image_color)
                
                # Check if the pothole is within the threshold distance.
                if pothole_info["depth_value"] < self.DISTANCE_THRESHOLD:
                    # Draw bounding box on the image
                    cv2.rectangle(image_color, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                    # # Display the image with bounding boxes
                    # cv2.imshow("YOLO Object Detection", image_color)
                    # cv2.waitKey(1)
                    
                    # Display image detection on Rviz.
                    image_with_boxes_msg = self.bridge.cv2_to_imgmsg(image_color, encoding="bgr8")
                    self.image_with_boxes_pub.publish(image_with_boxes_msg)
                    
                    # Save the pothole information to the detected pothole list.
                    self.detected_potholes.append(pothole_info)

                    # Increment the pothole counter
                    self.pothole_counter += 1
                    
                    # Publish marker for each pothole
                    self.publish_marker(pothole_info)
                    
                    # Print pothole information
                    print(f"\nPothole {self.pothole_counter} - \nOdom Coordinates: {pothole_info['odom_coords']}, \nSize: {pothole_info['box_size']} \nDepth Value: {pothole_info['depth_value']}")

        if self.stop_detection_flag:
            self.publish_status("Object detection stopped!")
    
    # Callback for camera information.           
    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    # Callback for depth image.
    def image_depth_callback(self, data):
        self.image_depth_ros = data

    # Callback for color image.
    def image_color_callback(self, data):
        # Wait for camera_model and depth image to arrive.
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # Covert images to open_cv.
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Perform YOLO object detection.
        results = self.model.predict(source=image_color, save_txt=True, save=False, stream=True)
        
        # Process and print pothole information.
        self.process_detected_potholes(results, image_depth, image_color)

    # Saves the detected potholes to a file
    def save_detected_potholes(self):
        count = 1
        try:
            with open(self.DETECTED_POTHOLE_FILE_PATH, 'w') as file:
                for pothole in self.detected_potholes:
                    file.write(f"{pothole}\n")
                    count += 1
        except Exception as e:
            print(f"Error writing to file: {e}")
        
        
# Main function to initialize and run the object detection node.
def main(args=None):
    # --- Initialize ROS
    rclpy.init(args=args)
    object_detection = ObjectDetector()
    
    # Run the node until the stop_detection_flag is set.
    while rclpy.ok() and not object_detection.stop_detection_flag:
        rclpy.spin_once(object_detection, timeout_sec=0.1)
    
    # Save detected potholes to a file.
    object_detection.save_detected_potholes()
    
    # Publish the final status and destroy the node.
    object_detection.publish_status("Object detection node stopped!")
    object_detection.destroy_node()
    
    # --- Shut down ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()