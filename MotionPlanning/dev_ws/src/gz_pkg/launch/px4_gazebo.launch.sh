# Make sure to give it execute permissions (chmod +x ./MotionPlanning/dev_ws/src/gz_pkg/launch/px4_gazebo.launch.sh).

#!/bin/bash

# Set the path to PX4 workspace
PX4_WS_DIR=~/PX4-Autopilot

# Set the name of custom robot model
ROBOT_MODEL_NAME=uas

# Set the path to custom robot model's SDF/URDF file
ROBOT_MODEL_FILE=$(pwd)/MotionPlanning/dev_ws/src/gz_pkg/models/uas/uas.sdf

# Change directory to your PX4 workspace
cd $PX4_WS_DIR

# Set the environment variable to specify your custom robot
export PX4_HOME_LAT=47.397742
export PX4_HOME_LON=8.545594
export PX4_HOME_ALT=0.0
export PX4_HOME=0
export PX4_ESTIMATOR=ekf2
export PX4_VEHICLE=$ROBOT_MODEL_NAME

# Launch PX4 SITL with your custom robot
make px4_sitl gazebo-classic_typhoon_h480

# Start Gazebo (if not already running)
if ! pgrep -x "gzserver" > /dev/null; then
  gazebo --verbose -s libgazebo_ros_init.so
fi