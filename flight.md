### Day In The Life Flight Instructions:

#### Connecting to the Jetson.
```bash
 sudo ssh attacks@10.42.0.1
```
password: 1234

#### Launch uXRCE-DDS Client
```
cd ~/dev/ATTACKS-SW/Micro-XRCE-DDS-Agent/build
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

#### Hover Mission:
```bash
cd ~/dev/ATTACKS-SW/scripts
sudo python3 ribbon_camera_record.py
```

```bash
cd ~/dev/ATTACKS-SW/ros_ws
colcon build
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch uas_bringup uas_hover_mission.launch.py
```

#### Waypoint Mission:
```bash
cd ~/dev/ATTACKS-SW/scripts
sudo python3 ribbon_camera_record.py
```

```bash
cd ~/dev/ATTACKS-SW/ros_ws
colcon build
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch uas_bringup uas_waypoint_mission.launch.py
```

#### Trailing Mission:
```bash
sudo su
cd ~/dev/ATTACKS-SW/scripts
source /opt/ros/foxy/setup.bash
python3 camera_publisher.py
```

```bash
cd ~/dev/ATTACKS-SW/ros_ws
colcon build
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch uas_bringup uas_trailing_mission.launch.py
```

#### File Transfer:
```bash
scp -r attacks@192.168.55.1:/home/attacks/dev/ATTACKS-SW/Datasets /Users/nicolasperrault/Desktop/Test
```
