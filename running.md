## Running the Simulation

### 1. Starting the Gazebo Simulation

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/PX4-Autopilot
  make px4_sitl gazebo-classic_uas__field
  ```


### 2. Creating a uXRCE-DDS Client

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/Micro-XRCE-DDS-Agent/build
  MicroXRCEAgent udp4 -p 8888
  ```

### 3. Launch the ROS UAS Mission Node

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/ros_ws
  colcon build
  source /opt/ros/foxy/setup.bash
  source install/local_setup.bash
  ros2 launch uas_bringup uas_complete_mission.launch.py
  ```



