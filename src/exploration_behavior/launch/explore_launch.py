# explore_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exploration_node',
            executable='exploration_node.py',
            output='screen',
            parameters=['exploration_params.yaml'],
        ),
    ])
