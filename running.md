## Running the Simulation

### 1. Starting the Gazebo Simulation

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/PX4-Autopilot
  make px4_sitl gazebo-classic_uas__field
  ```

- **Step 2:**
  ```bash
  param set MPC_XY_VEL_MAX 1.5
  param save
  ```
   ```
  *Note: This only needs to be ran the first time you are running the sim. If you wish to change the maximum horizontal velocity of the UAS you can reset it.*
  ```

### 2. Creating a uXRCE-DDS Client

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/Micro-XRCE-DDS-Agent/build
  MicroXRCEAgent udp4 -p 8888
  ```

### 3. Launch the ROS UAS Complete Mission Node

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/ros_ws
  colcon build
  source /opt/ros/foxy/setup.bash
  source install/local_setup.bash
  ros2 launch uas_bringup uas_complete_mission.launch.py
  ```

### Hover Mission Node:

  ```bash
  cd ~/dev/ATTACKS-SW/ros_ws
  colcon build
  source /opt/ros/foxy/setup.bash
  source install/local_setup.bash
  ros2 launch uas_bringup uas_hover_mission.launch.py
  ```

### Waypoint Mission Node:

  ```bash
  cd ~/dev/ATTACKS-SW/ros_ws
  colcon build
  source /opt/ros/foxy/setup.bash
  source install/local_setup.bash
  ros2 launch uas_bringup uas_waypoint_mission.launch.py
  ```

### Trailing Mission Node:

  ```bash
  cd ~/dev/ATTACKS-SW/ros_ws
  colcon build
  source /opt/ros/foxy/setup.bash
  source install/local_setup.bash
  ros2 launch uas_bringup uas_trailing_mission.launch.py
  ```




## Running HITL:

### On the high resource computer:
```bash
cd ~/dev/ATTACKS-SW/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_uas.world
```

- launch Q Ground Control and load parameter file if this the first time running HITL.

#### If a serial device is not detected:

- Make sure the high resource computer is connected to the cube orange via USB.
- Change `/dev/ttyACM0` to `/dev/ttyACM1` in
`PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/uas_hitl/uas_hitl.sdf`

### On the Jetson:
- Connect the Cube Orange from the Telem 2 port to a USB on the Jetson.
```bash
cd ~/dev/ATTACKS-SW/Micro-XRCE-DDS-Agent/build
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```
- The tty port will depend on which port it is connected to. To see all tty USB devices run the following command.

```
ls /sys/class/tty/ttyUSB*
```

- You should now see the ros topics populate.


```bash
cd ~/dev/ATTACKS-SW/ros_ws
colcon build
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch uas_bringup uas_hitl_mission.launch.py
```

### On the high resource computer:
- On Q Ground control press takeoff.
- Change flight mode to offboard.

### Help:

- If the vehicles state is unkown on Q Ground Control close the gazebo application and Q Ground. Unplug the Cube Orange and restart. 




 



