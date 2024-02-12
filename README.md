# ATTACKS
## Getting Setup: Ubuntu 20

- **1:** Installing ROS2 Foxy:

  ```bash
  python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  --user -U empy==3.3.4 pyros-genmsg setuptools \
  kconfiglib \
  future \
  ```

  ```bash
  sudo apt update && sudo apt upgrade -y
  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"
  sudo apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    libbullet-dev \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    ros-foxy-desktop \
    aptitude \
    gazebo libgazebo11 libgazebo-dev \
    ros-foxy-gazebo-ros-pkgs \
    python3-testresources \
    libimage-exiftool-perl \
  ```

  ```bash
  sudo apt-get install -y \
      libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python3-rospkg python3-jinja2 \
      libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y \
  ```

  ```bash
  nano ~/.bashrc
  ```
  - Add this line to the bottom of your bashrc
  ```bash
  source /opt/ros/foxy/setup.bash
  ```

  ```bash
  sudo apt remove cmake
  wget https://github.com/Kitware/CMake/releases/download/v3.28.1/cmake-3.28.1.tar.gz
  tar -zxvf cmake-3.28.1.tar.gz
  cd cmake-3.28.1/
  ./bootstrap
  make
  sudo make install
  ```

- **3:** Clone the repo in the `~/dev/` directory.

- **4:** Run the following commands to update the submodules:
  ```bash
  cd ~/dev/ATTACKS-SW
  git submodule update --init --recursive
  ```

- **4:** Run the following commands to update the PX4 autopilot:
  ```bash
  cd ~/dev/ATTACKS-SW/PX4-Autopilot
  git remote add upstream https://github.com/PX4/PX4-Autopilot.git
  git fetch upstream
  git fetch upstream --tags
  ```

- **5:** Run the following commands to make the Micro-XRCE-DDS-Agent:
  ```bash
  cd ~/dev/ATTACKS-SW/Micro-XRCE-DDS-Agent
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
  ```

## Running the Simulation

### 1. Starting the Gazebo Simulation

- **Step 1:** Open a new Ubuntu terminal.
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
  source /opt/ros/foxy/setup.bash
  source install/local_setup.bash
  ros2 launch uas_bringup uas_complete_mission.launch.py
  ```

## Sources
- PX4 and ROS2 installation tutorial: [ROS2 Communication](http://docs.px4.io/main/en/ros/ros2_comm.html)
- Code completion and debugging for ROS2 in vscode: [Code debugging](https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979)

