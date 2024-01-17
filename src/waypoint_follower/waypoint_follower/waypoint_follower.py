#!/usr/bin/env python3
"""
Waypoint Follower Node

This script initializes a ROS 2 (Robot Operating System 2) node for waypoint following using the
nav2_simple_commander package. It navigates a robot through predefined waypoints, waiting for
object detection to start before initiating the waypoint following task. The script is designed
to be executed in a ROS 2 environment.

Dependencies:
- ROS 2
- nav2_simple_commander package
- Python 3

Usage:
1. Ensure the ROS 2 environment is set up.
2. Execute this script to initiate the navigation task.
3. The robot will navigate through a sequence of waypoints, and the status will be published
   to the '/waypoint_follower/status' topic.

Author: Amel Varghese
Date: 11/01/2024
Github: https://github.com/RlNZLER/limo_ros2.git

References:
[1] Edouard Renard 2023, 'ROS2 Nav2 [Navigation 2 Stack] - with SLAM and Navigation', Udemy, viewed 23 December, 2023,
https://www.udemy.com/course/ros2-nav2-stack/

[2] Antonio Brandi 2023, 'Self Driving and ROS 2 - Learn by Doing! Odometry & Control', Udemy, viewed 23 December, 2023,
https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/

[3] OpenAI 2022, 'ChatGPT: OpenAI's Conversational Language Model', OpenAI, viewed 11 January 2024,
https://chat.openai.com/
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

SUCCEEDED = TaskResult.SUCCEEDED

class WaypointStatus(Node):
    
    def __init__(self):
        super().__init__("waypoint_status")
        
        # Publisher to broadcast the status of the waypoint follower.
        self.waypoint_follower_status_pub = self.create_publisher(String, '/waypoint_follower/status', 10)
        
        # Subscription to listen for object detection status.
        self.object_detection_status_sub = self.create_subscription(String, '/object_detection/status', self.waypoint_follower_callback, 10)
        
        # Flag to indicate when to start waypoint following.
        self.start_waypoint_following = False
        
    # Publishes the provided message as the status of the waypoint follower.
    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.waypoint_follower_status_pub.publish(msg)
        self.get_logger().info(msg.data)
        
    # Callback function to be executed when object detection status is received.
    # Sets the flag to start waypoint following when object detection is running.
    def waypoint_follower_callback(self, msg):
        if msg.data == "Object detection running!":
            self.start_waypoint_following = True

# Create a PoseStamped message from x, y, and theta (orientation).
def pose_from_xytheta(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

# Function to navigate to my origin point and return success or failure status.
def got_to_my_origin(status, nav):
    # --- Set initial pose
    initial_pose = pose_from_xytheta(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    
    # --- Wait for nav2 
    nav.waitUntilNav2Active()
    
    # --- To my origin
    # Waypoints to navigate from the current position to my origin.
    to_my_origin = [
        pose_from_xytheta(nav, 0.0, 0.0, 0.0),
        pose_from_xytheta(nav, 0.0, 0.08, 3.14),
        pose_from_xytheta(nav, -0.30, -0.10, 3.14),
        pose_from_xytheta(nav, -0.60, -0.10, 3.14),
        pose_from_xytheta(nav, -0.80, -0.10, 3.14),
        pose_from_xytheta(nav, -1.00, -0.10, 3.14),
        pose_from_xytheta(nav, -1.35, -0.10, 3.14),
        pose_from_xytheta(nav, -1.35, -0.10, 1.57),
        pose_from_xytheta(nav, -1.35, -0.10, -0.25)
    ]
    
    nav.followWaypoints(to_my_origin)
    
    # Loop to wait for the task to complete.
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        
    result = nav.getResult()
    
    if result == SUCCEEDED:
        status.publish_status("Reached my origin!")
        return True
    else:
        status.publish_status("Task Failed!")
        return False

# Function to follow a predefined set of waypoints and return success or failure status.
def follow_waypoints(status, nav):
    # Waypoints to navigate from my origin (at point A) and cover all positions and return to my origin.
    waypoints = [
        pose_from_xytheta(nav, -1.00, -0.10, -0.25), # From point A to point B.
        pose_from_xytheta(nav, -0.80, -0.10, 0.00),
        pose_from_xytheta(nav, -0.40, -0.10, 0.00),
        pose_from_xytheta(nav, 0.00, -0.10, 0.00),
        pose_from_xytheta(nav, 0.40, -0.10, 0.00),
        pose_from_xytheta(nav, 0.80, -0.12, 0.00),
        pose_from_xytheta(nav, 1.35, -0.12, 0.00),  # At point B.
        pose_from_xytheta(nav, 1.35, -0.12, -1.60), # From point B to point C.
        pose_from_xytheta(nav, 1.12, -0.40, -1.57),
        pose_from_xytheta(nav, 1.12, -0.80, -1.57),
        pose_from_xytheta(nav, 1.12, -1.00, -1.57), # Ar point C.
        pose_from_xytheta(nav, 1.12, -1.00, -3.14), # From point C to point D.
        pose_from_xytheta(nav, 1.00, -1.00, -3.14),
        pose_from_xytheta(nav, 0.80, -1.00, -3.14),
        pose_from_xytheta(nav, 0.40, -1.00, -3.14),
        pose_from_xytheta(nav, 0.00, -1.00, -3.14),
        pose_from_xytheta(nav, -0.40, -1.00, -3.14),
        pose_from_xytheta(nav, -0.80, -1.00, -3.14),
        pose_from_xytheta(nav, -1.05, -1.00, -3.14),
        pose_from_xytheta(nav, -1.05, -1.00, -3.14), # At point D.
        pose_from_xytheta(nav, -1.05, -1.00, -4.20), # From point D to point A.
        pose_from_xytheta(nav, -1.05, -0.50, -4.17),
        pose_from_xytheta(nav, -1.05, -0.10, -4.17),
        pose_from_xytheta(nav, -1.05, -0.10, 0.00),  # At point A.
        pose_from_xytheta(nav, -0.90, -0.10, 0.00),  # From point A to point E.
        pose_from_xytheta(nav, -0.60, -0.10, 0.00),
        pose_from_xytheta(nav, -0.20, -0.10, 0.00),  # At point E.
        pose_from_xytheta(nav, -0.20, -0.10, -1.57), # From point E to point F.
        pose_from_xytheta(nav, -0.20, -0.20, -1.57),
        pose_from_xytheta(nav, -0.20, -0.40, -1.57),
        pose_from_xytheta(nav, -0.20, -0.60, -1.57),
        pose_from_xytheta(nav, -0.20, -0.80, -1.57),
        pose_from_xytheta(nav, -0.20, -1.00, -1.57), # At point F.
        pose_from_xytheta(nav, -0.20, -1.00, -3.14), # From point F to point D.
        pose_from_xytheta(nav, -0.40, -1.00, -3.14),
        pose_from_xytheta(nav, -0.80, -1.00, -3.14),
        pose_from_xytheta(nav, -1.05, -1.00, -3.14),
        pose_from_xytheta(nav, -1.05, -1.00, -3.14), # At point D.
        pose_from_xytheta(nav, -1.05, -1.00, -4.20), # From point D to point A.
        pose_from_xytheta(nav, -1.05, -0.50, -4.17),
        pose_from_xytheta(nav, -1.05, -0.00, -4.17),
        pose_from_xytheta(nav, -1.05, -0.00, 0.00),  # At my origin.
        ]
        
    # --- Follow waypoints 
    status.publish_status("Waiting for object detection to start!")
    
    wait = True

    while wait and rclpy.ok():
        rclpy.spin_once(status)
        if status.start_waypoint_following:
            wait = False
        
    # status.publish_status("Object detection started!")
        
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()

    result = nav.getResult()
    if result == SUCCEEDED:
        status.publish_status("Task Completed!")
        return True
    else:
        status.publish_status("Task Failed!")
        return False

# Main function to initialize ROS, create navigator and status instances, and execute tasks.
def main():
    # --- Initialize ROS
    rclpy.init()
    nav = BasicNavigator()
    status = WaypointStatus()
    task_completed = False 
    
    reached_my_origin = got_to_my_origin(status, nav)
    
    if reached_my_origin:
        task_completed = follow_waypoints(status, nav)
        print ("Task Completed!")
    elif not reached_my_origin:
        print ("Task failed!")
    if not task_completed:
        print ("Task failed!")
    rclpy.spin(status)
    status.destroy_node()

    # --- Shut down ROS
    rclpy.shutdown()

if __name__ == "__main__":
    main()