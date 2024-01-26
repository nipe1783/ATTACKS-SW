# ATTACKS

## Getting Setup:

- **1:** Clone the repo in the `~/dev/` directory.
- **2:** Run the following commands to update the submodules:
  ```bash
  cd ~/dev/ATTACKS-SW
  git submodule update --init --recursive
  ```

- **3:** Run the following commands to update the PX4 autopilot:
  ```bash
  cd ~/dev/ATTACKS-SW/PX4-Autopilot
  git remote add upstream https://github.com/PX4/PX4-Autopilot.git
  git fetch upstream
  git fetch upstream --tags
  ```

- **4:** Run the following commands to make the Micro-XRCE-DDS-Agent:
  ```bash
  cd ~/dev/ATTACKS-SW/Micro-XRCE-DDS-Agent
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
  ```

## Dependencies:

ROS2 Humble. To install, run the following commands:
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
sudo apt remove gz-garden
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
sudo apt install ros-humble-gazebo-ros-pkgs
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```
```bash
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

## Running the Simulation

### 1. Starting the Gazebo Simulation

- **Step 1:** Op
en a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/PX4-Autopilot
  make px4_sitl gazebo-classic_uas__field
  ```

- **Step 2:**
  ```bash
  gimbal test pitch -90
  ```

- **Step 3:**
  ```bash
  param set MPC_XY_VEL_MAX 3
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

### 3. Launch the ROS UAS Mission Node

- **Step 1:** Open a new Ubuntu terminal.
  ```bash
  cd ~/dev/ATTACKS-SW/ros_ws
  colcon build
  source /opt/ros/humble/setup.bash
  source install/local_setup.bash
  ros2 launch uas_complete_mission uas_complete_mission.launch.py
  ```

## Sources
- PX4 and ROS2 installation tutorial: [ROS2 Communication](http://docs.px4.io/main/en/ros/ros2_comm.html)
- Code completion and debugging for ROS2 in vscode: [Code debugging](https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979)