# Make sure to give it execute permissions (chmod +x ./MotionPlanning/dev_ws/src/gz_pkg/launch/px4_gazebo.launch.sh).

#!/bin/bash

# Set the path to PX4 workspace
PX4_WS_DIR=~/dev/ATTACKS-SW/MotionPlanning/PX4-Autopilot

# Set the name of custom robot model
ROBOT_MODEL_NAME=uas

# Set the path to custom robot model's SDF/URDF file
# ROBOT_MODEL_FILE=$(pwd)/MotionPlanning/dev_ws/src/gz_pkg/models/uas/uas.sdf

# Change directory to your PX4 workspace
cd $PX4_WS_DIR

killall gzserver
killall gzclient

# Launch PX4 SITL with your custom robot
# make px4_sitl gazebo-classic_typhoon_h480
make px4_sitl gazebo-classic_uas

# Start Gazebo (if not already running)
if ! pgrep -x "gzserver" > /dev/null; then
  gazebo --verbose -s libgazebo_ros_init.so
fi