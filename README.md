# Limo Robot - Waypoint Following, Object Detection, and Report Generation

## Overview

This project focuses on implementing a limo robot capable of waypoint following, object detection, and generating reports based on the collected data. The robot utilizes advanced algorithms to navigate through predefined waypoints, detect objects in its environment, and produce insightful reports.

## Features

1. **Waypoint Following:**
   - The limo robot autonomously follows a predefined set of waypoints.
   - Navigation algorithms ensure efficient and safe movement between waypoints.

2. **Object Detection:**
   - Utilizes computer vision techniques (YOLOv8n) to detect and identify objects in the robot's surroundings.
   - Object detection is performed in real-time during the robot's operation.

3. **Report Generation:**
   - Gathers data during navigation and object detection processes.
   - Generates comprehensive report based on the collected and filtered data.
   - Report include pothole information such as number of potholes, pothole odom coordinates, map coordinates, size and a map showing the location of each pothole.

## Run
```bash
$ cd limo_ros2
$ python3 limo_launch.py
```
You can change the real-pothole world simulation and simple-pothole world simulation by changing the variable "REAL_WORLD" in the limo_launch.py file.

## Changelog

### 2023-08-22

- Moved obstacle layer to local costmap
- Cleaned up old ros1 costmap params
- **Set environment flag to use cyclone dds to fix costmap obstacle not working**

## Setup development environment

You can set up the development environment either on the Jetson Nano or your desktop/laptop PC. The steps for the setup are the same:

* **Clone the repo to the computer**

```bash
$ git clone https://github.com/westonrobot/limo_ros2.git
$ cd limo_ros2
```

* **Install VS Code with [Remote Development Plugins](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)**

* **Start development inside the container**
    If you're developing on the LIMO directly:
    1. Launch VS Code on LIMO
    2. In the workspace root folder, "Crtl + Shift + P" to open up the command palette, and select "Remote-Containers: Rebuild and Reopen in Container"

    If you're developing remotely from your PC:
    1. Launch VS Code on your PC
    2. In the workspace root folder, "Crtl + Shift + P" to open up the command palette, and select "Remote-SSH: Connect to Host", finish the connection configuration
    3. Type "Crtl + Shift + P" to open up the command palette again, and select "Remote-Containers: Rebuild and Reopen in Container"

Latest image: westonrobot/limo_ros:humble_22082023

You can find more information about development inside containers [here](./docs/README.md)

## Use limo_ros2 packages

Once you're inside the container, the commands to build or launch ROS packages are the same as those in a native environment:

* Build the colcon workspace
  
    ```bash
    # in workspace root
    ./build.sh

    # Remember to always source ros workspace on new terminal
    . install/setup.bash
    ```

* Bringup robot

    ```bash
    # Bringup robot
    ros2 launch limo_bringup limo_start.launch.py

    # OR

    # Bringup robot in simulator
    # Add use_sim_time:=true argument for subsequent launches if using sim
    ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
    ```

* Bringup camera
  
    ```bash
    ros2 launch astra_camera dabai.launch.py
    ```

* Teleoperation

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

* SLAM

    ```bash
    # Launch SLAM node
    ros2 launch limo_bringup cartographer.launch.py

    # To save map
    ros2 run nav2_map_server map_saver_cli -f [filename]
    ```

* Navigation

    ```bash
    # In seperate terminals
    ros2 launch limo_navigation limo_localization.launch.py

    ros2 launch limo_navigation limo_controller.launch.py

    # Single launch
    ros2 launch limo_navigation limo_navigation.launch.py
    ```

## Running your own navigation stack or simulation on local PC

**Note: Limo container is running on cyclone DDS, you need to set the following environment variable**

```bash
# Install package
sudo apt install ros-humble-rmw-cyclonedds-cpp
# Export environment variable
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Sample gazebo and navigation are provided for differential motion model in the repo, to use them

    * Clone the repo into your local pc
    * Feel free to use provided Dockerfile as a reference
    * Rebuild new container for local PC

## Displaying GUI from Docker Machine On LIMO Screen

To get display out of docker and on the LIMO screen;

- On the host machine, allow all connections with:
```bash
xhost +
```

- On the docker machine:
```bash
# export the display
export DISPLAY=:0
```

## Viewing the output of the camera 
Note: Tested on Ubuntu 22 (POP) and Jetpack 4.6 w/docker

To get over the permission error, WITH THE CAMERA UNPLUGGED

```bash
cd limo_ros2/src/ros2_astra_camera/astra_camera/scripts
sudo cp 56-orbbec-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

This example rviz workspace has everything working.

```bash
ros@agilex:/workspaces/limo_ros2/src/ros2_astra_camera/astra_camera/rviz$ rviz2 -d pointcloud.rviz 
```

## Process for getting to teleop with the cameras diplaying in rviz2 on LIMO

In addition to the steps above regarding udev rules and exporting the displays;

- Remote into the LIMO DevContainer

- Launch the LIMO Base
```bash
ros@agilex:/workspaces/limo_ros2$ ros2 launch limo_bringup limo_start.launch.py
```
- Bringup the Camera
```bash
ros2 launch astra_camera dabai.launch.py
```
- Run the provided rviz2 configuration
```bash
rviz2 -d src/ros2_astra_camera/astra_camera/rviz/pointcloud.rviz 
```
- Run Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- At this point a ```ros2 topic list``` gives;
```bash
os@agilex:/workspaces/limo_ros2/$ ros2 topic list
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressedDepth
/camera/color/image_raw/theora
/camera/depth/camera_info
/camera/depth/color/points
/camera/depth/image_raw
/camera/depth/image_raw/compressed
/camera/depth/image_raw/compressedDepth
/camera/depth/image_raw/theora
/camera/depth/points
/camera/extrinsic/depth_to_color
/camera/ir/camera_info
/camera/ir/image_raw
/camera/ir/image_raw/compressed
/camera/ir/image_raw/compressedDepth
/camera/ir/image_raw/theora
/cmd_vel
/imu
/joint_states
/limo_status
/odom
/parameter_events
/robot_description
/rosout
/scan
/tf
/tf_static
/ydlidar_ros2_driver_node/transition_event
```

Notes. 
- The above procedure may have redundant steps. Unsure if camera bringup needs to be separate. 
- Haven't tested this with a camera subscriber node yet.

To Do:
- Hardware Camera - Compare example .rviz file above with the one we want to use normally. might have to make one.
- test camera with a subscriber node, colour chaser etc
- Add ```source install/setup.bash``` to bashrc file
- Check bashrc is mapped properly
- Check everything is being installed on startup iptools tree etc

## Issues:

### Jeston Crashed - Possibly overheating. Was remoting in with a display scaled to my laptop. everything running! Didn't happen for the rest of session after restart

### Getting errors  about QOS incompatibility.

- As a warning with the main bringup file
  ```bash
  ydlidar_ros2_driver_node-4] [WARN] [1695551352.042005881] [ydlidar_ros2_driver_node]: New subscription discovered on topic '/scan', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
  ```

- Got the same sort of warning when connecting to the camera
```bash 
[WARN] [1695553761.805006031] [rviz]: New publisher discovered on topic '/camera/depth/image_raw', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```
[SOLVED] - https://github.com/ros-visualization/rqt/issues/187#issuecomment-810105698

![](https://user-images.githubusercontent.com/10749928/112974428-28705a00-90ee-11eb-85b3-3787bc8c781a.png)
