import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from rclpy.duration import Duration

SUCCESS_THRESHOLD = 5
SUCCEEDED = TaskResult.SUCCEEDED
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

def move_autonomously():
    rclpy.init()

    nav = BasicNavigator()

    # Set the initial pose (you can adjust the initial position and orientation)
    initial_pose = pose_from_xytheta(nav, 0.0, 1.0, 0.0)
    nav.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    nav.waitUntilNav2Active()

    # Move to a goal position
    vacant_poses = [
        pose_from_xytheta(nav, 0.0, 0.0, 0.0),
        pose_from_xytheta(nav, 0.0, 0.0, 3.14),
        pose_from_xytheta(nav, 1.0, 0.0, 0.0),
        pose_from_xytheta(nav, 1.0, 0.0, 3.14),
        pose_from_xytheta(nav, 2.0, 0.0, 0.0),
        pose_from_xytheta(nav, 2.0, 0.0, 3.14),
        pose_from_xytheta(nav, 3.0, 0.0, 0.0),
        pose_from_xytheta(nav, 3.0, 0.0, 3.14),
        pose_from_xytheta(nav, 4.0, 0.0, 3.14),
        pose_from_xytheta(nav, 4.0, 1.0, 0.0),
        pose_from_xytheta(nav, 3.0, 1.0, 3.14),
        pose_from_xytheta(nav, 3.0, 1.0, 0.0),
        pose_from_xytheta(nav, 2.0, 1.0, 3.14),
        pose_from_xytheta(nav, 2.0, 1.0, 0.0),
        pose_from_xytheta(nav, 1.0, 1.0, 3.14),
        pose_from_xytheta(nav, 1.0, 1.0, 0.0),
        pose_from_xytheta(nav, 0.0, 1.0, 3.14),
        pose_from_xytheta(nav, 0.0, 1.0, 0.0),
        pose_from_xytheta(nav, 0.0, 0.0, 0.0)]
    
    success = 0
    while success < SUCCESS_THRESHOLD:
        success = 0
        for i, goal_pose in enumerate(vacant_poses):
            nav.goToPose(goal_pose)

            # Wait until the navigation task is complete
            while not nav.isTaskComplete():
                feedback = nav.getFeedback()


            # Print the result
            result = nav.getResult()
            print(result)
            if result == SUCCEEDED:
                success += 1
                print('Point %d found! %i' % (i, success))
                if success == SUCCESS_THRESHOLD:
                    break  # Exit the loop when the success threshold is reached
            else:
                success = 0 
                print('Localization attempt failed!')


    # move to origin
    origin_pose = pose_from_xytheta(nav, 0.0, 0.0, 0.0)
    nav.goToPose(origin_pose)
    
    # Wait until the navigation task is complete
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        
    # Shutdown
    nav.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    move_autonomously()