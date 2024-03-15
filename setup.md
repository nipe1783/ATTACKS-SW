## Getting Setup: Ubuntu 20

- **1:** Installing Dependencies:
    ```bash
    sudo apt update && sudo apt install locales
    sudo apt-get update
    sudo apt-get upgrade
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
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
    pip3 install kconfiglib
    pip3 install future
    pip3 install --user pyros-genmsg
    ```
    ```bash
    sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python3-rospkg python3-jinja2
    ```
    ```bash
    sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
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
    ```bash
    sudo apt install libyaml-cpp-dev
    sudo apt install libimage-exiftool-perl
    sudo apt install aptitude
    sudo apt install ros-dev-tools
    sudo apt install ros-foxy-desktop
    sudo apt-get update
    sudo apt-get install ros-foxy-gazebo-ros-pkgs
    ```

    - Enter `nano ~/.bashrc` and add `source /opt/ros/foxy/setup.bash` to the bottom.
    - Close and open a new Ubuntu terminal.
    - Clone the repo in your `~/dev` directory.
    ```bash
    cd ~/dev/ATTACKS-SW
    git submodule update --init --recursive
    ```

- **2:** Setting up Gazebo sim:
    - Open a new Ubuntu terminal.

    ```bash
    cd ~/dev/ATTACKS-SW/PX4-Autopilot
    git remote add upstream https://github.com/PX4/PX4-Autopilot.git
    git fetch upstream
    git fetch upstream --tags
    ```

    ```bash
    cd ~/dev/ATTACKS-SW/PX4-Autopilot
    make px4_sitl_rtps gazebo_uas__field
    ```

    - The Gazebo sim should now be running.
    - Go to the Gazebo terminal.

    ```bash
    param set MPC_XY_VEL_MAX 2
    param set MC_PITCHRATE_MAX 35
    param set MC_ROLLRATE_MAX 35
    param save
    ```
