# ATTACKS

## Getting Setup: Ubuntu 20

- **1:** Installing ROS2 Foxy:

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

```bash
sudo apt update && sudo apt install -y \
libbullet-dev \
python3-pip \
python3-pytest-cov \
ros-dev-tools
```

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
pytest
```

```bash
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
```

```bash
sudo apt install --no-install-recommends -y \
  libcunit1-dev
```

```bash
mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos src
```

```bash
sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"
```

```bash
cd ~/ros2_foxy/
colcon build --symlink-install
```

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ros-foxy-desktop
sudo apt install ros-dev-tools
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python3-rospkg python3-jinja2
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python3-rospkg python3-jinja2
sudo apt remove gz-garden
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install python3-testresources
```

```bash
nano ~/.bashrc
```

Add this line to the bottom:
```bash
source /opt/ros/foxy/setup.bash
```

- **2:** Clone the repo in the `~/dev/` directory.

- **3:** Run the following commands to update the submodules:
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

## Getting Setup: Ubuntu 22

- **1:** Installing ROS2 Humble:


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
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python3-rospkg python-jinja2
sudo apt remove gz-garden
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install python3-testresources
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```

```bash
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
pip3 install kconfiglib
pip3 install future
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
  source /opt/ros/foxy/setup.bash
  source install/local_setup.bash
  ros2 launch uas_bringup uas_complete_mission.launch.py
  ```

## Sources
- PX4 and ROS2 installation tutorial: [ROS2 Communication](http://docs.px4.io/main/en/ros/ros2_comm.html)
- Code completion and debugging for ROS2 in vscode: [Code debugging](https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979)

