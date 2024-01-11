import subprocess
import time

REAL_WORLD = False
def run_command_in_terminal(command):
    # Open a new terminal and run the specified command in the current directory
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'{command}; exec bash'])

def main():
    # Command 2: colcon build
    run_command_in_terminal('colcon build')
    time.sleep(10)

    # Command 3: source the workspace
    run_command_in_terminal('. install/setup.bash')
    time.sleep(3)

    # Command 4: launch Gazebo
    if REAL_WORLD:
        run_command_in_terminal('ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes.world')
    else:
        run_command_in_terminal('ros2 launch limo_gazebosim limo_gazebo_diff.launch.py')
    time.sleep(15)

    # Command 5: launch navigation
    run_command_in_terminal('ros2 launch limo_navigation limo_navigation.launch.py')
    time.sleep(10)
    
    # Command 8: run report generator
    run_command_in_terminal('ros2 run report_generator report_generator')
    
    # Command 6: run object detection
    run_command_in_terminal('ros2 run object_detection object_detection')
    
    # Command 7: run waypoint following
    run_command_in_terminal('ros2 run waypoint_follower waypoint_follower')
    time.sleep(30)
    
if __name__ == "__main__":
    main()
