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
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

# Plotting the pothole positions
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

class ObjectDetector(Node):
    camera_model = None
    image_depth_ros = None
    odom_coords_list = []

    visualisation = True
    color2depth_aspect = 1.0

    def __init__(self):    
        super().__init__('image_projection_3')
        self.bridge = CvBridge()

        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.camera_info_callback, 
                                                qos_profile=qos.qos_profile_sensor_data)
        
        self.object_location_pub = self.create_publisher(PoseStamped, '/limo/object_location', 10)

        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', 
                                                  self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', 
                                                  self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.shutdown_requested = False
        self.timer = self.create_timer(1.0, self.check_shutdown)
        
    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        if self.camera_model is None or self.image_depth_ros is None:
            return

        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        image_mask = cv2.inRange(image_color, (180, 0, 180), (230, 30, 230))
        cv2.imshow("Binary Mask", image_mask)
        cv2.waitKey(1)

        M = cv2.moments(image_mask)

        if M["m00"] == 0:
            print('No object detected.')
            return

        image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
        depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect, 
            image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect)
        depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]

        print('image coords: ', image_coords)
        print('depth coords: ', depth_coords)
        print('depth value: ', depth_value)

        camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0]))
        camera_coords = [x/camera_coords[2] for x in camera_coords]
        camera_coords = [x*depth_value for x in camera_coords]

        print('camera coords: ', camera_coords)

        object_location = PoseStamped()
        object_location.header.frame_id = "depth_link"
        object_location.pose.orientation.w = 1.0
        object_location.pose.position.x = camera_coords[0]
        object_location.pose.position.y = camera_coords[1]
        object_location.pose.position.z = camera_coords[2]

        self.object_location_pub.publish(object_location)        

        transform = self.get_tf_transform('depth_link', 'odom')
        p_camera = do_transform_pose(object_location.pose, transform)
        
        odom_coords = p_camera.position
        print('odom coords: ', odom_coords)
        
        self.odom_coords_list.append(odom_coords)

        if self.visualisation:
            cv2.circle(image_color, (int(image_coords[1]), int(image_coords[0])), 10, 255, -1)
            cv2.circle(image_depth, (int(depth_coords[1]), int(depth_coords[0])), 5, 255, -1)

            image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
            image_depth *= 1.0/10.0

            cv2.imshow("image depth", image_depth)
            cv2.imshow("image color", image_color)
            cv2.waitKey(1)
            
    def check_shutdown(self):
        if self.shutdown_requested:
            print("Shutting down...")
            self.display_and_exit()

    def display_and_exit(self):
        pothole_points = []
        for coord in self.odom_coords_list:
            pothole_points.append([coord.x, coord.y, coord.z])

        self.cluster_potholes(pothole_points)

    def cluster_potholes(self, pothole_points):
        pothole_points_std = StandardScaler().fit_transform(pothole_points)
        dbscan = DBSCAN(eps=0.5, min_samples=2)
        labels = dbscan.fit_predict(pothole_points_std)

        unique_labels = set(labels)
        duplicate_clusters = []

        for label in unique_labels:
            cluster_indices = np.where(labels == label)[0]
            if len(cluster_indices) > 1:
                duplicate_clusters.append(cluster_indices)

        for i, cluster in enumerate(duplicate_clusters, 1):
            print(f"Duplicate Cluster #{i}:")
            for idx in cluster:
                print(f"Pothole #{idx + 1}: {pothole_points[idx]}")
            print()

        # Plot the points in 3D
        self.plot_3d_points(pothole_points)

        self.destroy_node()
        rclpy.shutdown()
        exit()

    def plot_3d_points(self, pothole_points):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        x_coords = [coord[0] for coord in pothole_points]
        y_coords = [coord[1] for coord in pothole_points]
        z_coords = [coord[2] for coord in pothole_points]

        ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')

        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        plt.show()

def main(args=None):
    try:
        rclpy.init(args=args)
        image_projection = ObjectDetector()
        rclpy.spin(image_projection)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        image_projection.display_and_exit()

if __name__ == '__main__':
    main()
