# ATTACKS Motion Planning
### Overview:
This directory contains all code, documentation and resources relevent to the missions motion and task planning. This includes information on ROS2, Gazebo, PX4, motion planning algorithms and our task planning structure. 


### Running the simulation:

Follow this tutorial: `http://docs.px4.io/main/en/ros/ros2_comm.html` to install PX4 and ROS2. 

1. Start the gazebo simulation.
```
cd ~/dev/ATTACKS-SW/MotionPlanning/PX4-Autopilot
make px4_sitl gazebo-classic_uas
```

Note: If this gives CMake Errors, CD into tho PX4-Autopilot dir and run:
```
git remote add upstream https://github.com/PX4/PX4-Autopilot.git
git fetch upstream
git fetch upstream --tags
```

2. Create a uXRCE-DDS client. This creates the "tunnel" from PX4 autopilot to Ros. 
```
cd ~/Micro-XRCE-DDS-Agent/build
MicroXRCEAgent udp4 -p 8888
```

3. Launch the ROS listener node. This prints the current state of the UAS. Confirming we are simulating data in gazebo, using the PX4 autopilot, moving the data from PX4 to ROS using the DDS to transfer data, and recieving the data in our ROS node.
```
cd ~/dev/ATTACKS-SW/MotionPlanning/ros_ws
colcon build // run this if it is your first time building the ros2 code.
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

### Launching basic gazebo sim w/ diff drive robot:
```
cd ~/dev/ATTACKS-SW/MotionPlanning/ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch gz_pkg launch_sim.launch.py
```

### Launching basic gazebo sim w/ UAS:
```
cd ~/dev/ATTACKS-SW/MotionPlanning/ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch gz_pkg rsp_sdf.launch.py
```