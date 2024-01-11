#!/usr/bin/env python3
########################################################################################################################################################
# LIMO LAUNCH
#
# This Python script automates the process of setting up and launching a robotic simulation environment using ROS 2 (Robot Operating System 2) 
# on a Linux system. The script utilizes the gnome-terminal command to open multiple terminal windows and execute different ROS 2 commands sequentially.
#
########################################################################################################################################################

# Time module for introducing delays and subprocess module for running terminal commands from the script.
import time
import subprocess

# Toggle the real-pothole world simulation (True) and simple-pothole world simulation (False).
REAL_WORLD = True

def run_command_in_terminal(command):
    # Open a new terminal and run the specified command in the current directory
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'{command}; exec bash'])

def main():
    # Command 1: colcon build.
    run_command_in_terminal('colcon build')
    time.sleep(10)

    # Command 2: source the workspace.
    run_command_in_terminal('. install/setup.bash')
    time.sleep(3)

    # Command 3: launch gazebo simulation node.
    if REAL_WORLD:
        run_command_in_terminal('ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes.world')
    else:
        run_command_in_terminal('ros2 launch limo_gazebosim limo_gazebo_diff.launch.py')
    time.sleep(15) # Wait for simulation to start.

    # Command 4: launch navigation node.
    run_command_in_terminal('ros2 launch limo_navigation limo_navigation.launch.py')
    time.sleep(10) # Wait for navigation and amcl to start.
    
    # Command 5: run report generator node.
    run_command_in_terminal('ros2 run report_generator report_generator')
    
    # Command 6: run object detection node.
    run_command_in_terminal('ros2 run object_detection object_detection')
    
    # Command 7: run waypoint following node.
    run_command_in_terminal('ros2 run waypoint_follower waypoint_follower')
    
if __name__ == "__main__":
    main()
