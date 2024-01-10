
# ATTACKS Motion Planning

## Overview
This directory contains all code, documentation, and resources relevant to the mission's motion and task planning. It includes information on ROS2, Gazebo, PX4, motion planning algorithms, and our task planning structure.

## Running the Simulation

### 1. Starting the Gazebo Simulation

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/MotionPlanning/PX4-Autopilot
  make px4_sitl gazebo-classic_uas__field

  ```
  *Note: If you encounter CMake errors, navigate to the PX4-Autopilot directory and run the following commands:*
  ```bash
  git remote add upstream https://github.com/PX4/PX4-Autopilot.git
  git fetch upstream
  git fetch upstream --tags
  ```

### 2. Creating a uXRCE-DDS Client

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/Micro-XRCE-DDS-Agent/build
  MicroXRCEAgent udp4 -p 8888
  ```

### 3. Launch the ROS Listener Node

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/MotionPlanning/ros_ws
  colcon build
  source /opt/ros/humble/setup.bash
  source install/local_setup.bash
  ros2 launch ctrl_pkg UAS_state_listener.launch.py
  ```
- **Step 2:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/MotionPlanning/ros_ws
  colcon build
  source /opt/ros/humble/setup.bash
  source install/local_setup.bash
  ros2 launch ctrl_pkg UAS_sensor_listener.launch.py
  ```

  **Step 3:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/MotionPlanning/ros_ws
  colcon build
  source /opt/ros/humble/setup.bash
  source install/local_setup.bash
  ros2 launch uas_trailing uas_trailing.launch.py
  ```

### 4. Launch the ROS Control Node

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/MotionPlanning/ros_ws
  colcon build
  source /opt/ros/humble/setup.bash
  source install/local_setup.bash
  ros2 run px4_ros_com offboard_control
  ```

## Sources
- PX4 and ROS2 installation tutorial: [ROS2 Communication](http://docs.px4.io/main/en/ros/ros2_comm.html)
- Code completion and debugging for ROS2 in vscode: [Code debugging](https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979)
