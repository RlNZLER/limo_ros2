import subprocess
import time

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
    run_command_in_terminal('ros2 launch limo_gazebosim limo_gazebo_diff.launch.py')

    # Add a delay to make sure the Gazebo launch has started before launching the navigation
    time.sleep(15)  # Adjust the delay as needed

    # Command 5: launch navigation
    run_command_in_terminal('ros2 launch limo_navigation limo_navigation.launch.py')

if __name__ == "__main__":
    main()
