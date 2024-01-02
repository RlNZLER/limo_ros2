#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


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

def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()
    
    # --- Set initial pose
    initial_pose = pose_from_xytheta(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    
    # --- Wait for nav2 
    nav.waitUntilNav2Active()
    
    # --- Send nav2 go
    waypoints = [
        pose_from_xytheta(nav, 0.0, 0.08, 3.14,),
        pose_from_xytheta(nav, -0.30, -0.10, 3.14),
        pose_from_xytheta(nav, -0.60, -0.10, 3.14),
        pose_from_xytheta(nav, -0.80, -0.10, 3.14),
        pose_from_xytheta(nav, -1.00, -0.10, 3.14),
        pose_from_xytheta(nav, -1.35, -0.10, 3.14),
        pose_from_xytheta(nav, -1.35, -0.10, 1.57),
        pose_from_xytheta(nav, -1.35, -0.10, -0.25), # At my origin (point A).
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
        pose_from_xytheta(nav, -1.05, -0.00, -4.17),
        pose_from_xytheta(nav, -1.05, -0.00, 0.00),  # At point A.
        pose_from_xytheta(nav, -0.90, -0.00, 0.00),  # From point A to point E.
        pose_from_xytheta(nav, -0.60, -0.00, 0.00),
        pose_from_xytheta(nav, -0.20, -0.00, 0.00),  # At point E.
        pose_from_xytheta(nav, -0.20, -0.00, -1.57), # From point E to point F.
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
        pose_from_xytheta(nav, -1.05, -0.00, 0.00),  # At point A.
        ]

    # --- Go to one pose
    # nav.goToPose(goal_pose1)
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # print(feedback)
        
    # --- Follow waypoints
    # waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4]      
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()

    print(nav.getResult())
    
    
    # --- Shut down
    rclpy.shutdown()

if __name__ == "__main__":
    main()