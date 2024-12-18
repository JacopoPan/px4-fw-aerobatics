# PX4 Fixed-Wing Aerobatics

Autonomous aileron rolls and dives using VTOL's PX4 off-board control in ROS2

<img src="/resources/roll-and-dive.gif" alt="Roll and Dive" width="800"/>

## Installation

Tested on Ubuntu 22.04 LTS with Nvidia 535 driver

### ROS2 Humble (1 of 7)

```sh
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
sudo apt-get install ros-humble-bondcpp ros-humble-ament-cmake-clang-format # Missing dependencies
source /opt/ros/humble/setup.bash
```

Optionally, add ros2's `setup.bash` to `~/.bashrc`

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Gazebo Harmonic (2 of 7)

```sh
sudo apt-get update
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
sudo apt install ros-humble-ros-gzharmonic # To publish the Gazebo /clock topic used by ROS2 when use_sim_time is true
```

### PX4 SITL (3 of 7)

```sh
mkdir -p ~/git/
cd ~/git/ # Place in a folder for git repos
git clone git@github.com:JacopoPan/PX4-Autopilot.git # Fork of 1.14.3 with custom vtol takeoff heading and Gazebo Harmonic fix
cd PX4-Autopilot/
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh --no-sim-tools # Reboot
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
make px4_sitl
```

### QGroundControl (4 of 7)

```sh
cd ~/ # Place in the home folder
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage # From https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
chmod +x ./QGroundControl.AppImage # Logout and login
```

### uXRCE DDS Agent (5 of 7)

```sh
cd ~/git/ # Place in a folder for git repos
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent/
mkdir build
cd build/
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### ROS2 Workspace (6 of 7)

```sh
mkdir -p ~/git/ws/src/
cd ~/git/ws/src/ # Place in the source folder of a workspace inside the git folder
git clone -b release/1.14 https://github.com/PX4/px4_msgs.git # PX4 messages definitions
git clone git@github.com:JacopoPan/px4-fw-aerobatics.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build
```

Optionally, add the workspace's `setup.bash` to `~/.bashrc`

```sh
echo "source ~/git/ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Customize PX4 SITL (7 of 7)

#### Customize PX4's DDS Topics for px4-fw-aerobatics

```sh
cp ~/git/ws/src/px4-fw-aerobatics/resources/dds_topics.yaml ~/git/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml 
```

#### Embed Custom ROMS Files in the SITL Firmware Binaries

These include custom PX4 `params` for initial GPS position and do not quadchute during aggressive maneuvers
```sh
cp -r ~/git/ws/src/px4-fw-aerobatics/resources/airframes ~/git/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/
```
Update the airframes' `CMakeLists.txt`
```sh
cd ~/git/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
rm CMakeLists.txt
echo "px4_add_romfs_files(" >> CMakeLists.txt && ls | sed 's/^/  /' >> CMakeLists.txt && echo ")" >> CMakeLists.txt
```

Rebuild PX4 SITL

```sh
cd ~/git/PX4-Autopilot
rm -rf build/
make px4_sitl # There will be warnings for the custom_vtol .sdf not being within PX4-Autopilot 
```

Or use `./resources/set_dds_roms_and_rebuild_px4.sh` to apply these changes of PX4 params or DDS topics

---

## Example Startup

In one terminal, start the DDS `MicroXRCEAgent`

```sh
MicroXRCEAgent udp4 -p 8888
```

In a second terminal, start the Gazebo simulation

```sh
GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/git/ws/src/px4-fw-aerobatics/resources/models gz sim -r ~/git/ws/src/px4-fw-aerobatics/resources/worlds/default.sdf
# Set the path where to find the drones' and objects .sdf and autostart (with `-r`) the default simulation world
```

In a third terminal, bridge the Gazebo clock to ROS2

```sh
source ~/git/ws/install/setup.bash && source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

In a fourth terminal, start PX4 SITL

```sh
# Export LAT_DEG, LON_DEG, ELEV_M from the world .sdf
# Set the PX4_GZ_MODEL_POSE 'x,y,z,r,p,y' within the world,
# Select the vehicle configuration PX4_SYS_AUTOSTART (4008 for custom_vtol)
# Set the DDS client namespace and port, PX4_UXRCE_DDS_NS, PX4_UXRCE_DDS_PORT
LAT_DEG=$(awk -F '[><]' '/<spherical_coordinates>/,/<\/spherical_coordinates>/' ~/git/ws/src/px4-fw-aerobatics/resources/worlds/default.sdf | awk -F '[><]' '/latitude_deg/ {print $3}') LON_DEG=$(awk -F '[><]' '/<spherical_coordinates>/,/<\/spherical_coordinates>/' ~/git/ws/src/px4-fw-aerobatics/resources/worlds/default.sdf | awk -F '[><]' '/longitude_deg/ {print $3}') ELEV_M=$(awk -F '[><]' '/<spherical_coordinates>/,/<\/spherical_coordinates>/' ~/git/ws/src/px4-fw-aerobatics/resources/worlds/default.sdf | awk -F '[><]' '/elevation/ {print $3}') PX4_GZ_MODEL_POSE='0,0,0,0,0,0' PX4_SYS_AUTOSTART=4008 PX4_UXRCE_DDS_NS="Drone1" PX4_UXRCE_DDS_PORT=8888 ~/git/PX4-Autopilot/build/px4_sitl_default/bin/px4 # add `-i 1` (2,3,..) for additional instances of PX4
```

Start QGroundControl in a fifth terminal for vehicle monitoring

```sh
cd ~/
./QGroundControl.AppImage
```

Alternatively, run all of the above 5 steps in one `screen` session

```sh
screen -c ~/git/ws/src/px4-fw-aerobatics/resources/screenrc-example
killall screen && pkill -f gz # To quit screen and gazebo
```

## Do VTOL Aerobatics

Start the simulation environment for `custom_vtol` as in the example above

```sh
screen -c ~/git/ws/src/px4-fw-aerobatics/resources/screenrc-example
```

Run `PX4Whisperer` (a simple node implementing a service for autonomous takeoff, landing, and offboard maneuvers)

```sh
cd ~/git/ws/src/px4-fw-aerobatics/vtol_example/
source /opt/ros/humble/setup.bash && source ~/git/ws/install/setup.bash 
python3 px4whisperer.py
```

Run the example script with a complete mission including an aileron roll and a dive

```sh
cd ~/git/ws/src/px4-fw-aerobatics/vtol_example
source /opt/ros/humble/setup.bash && source ~/git/ws/install/setup.bash 
./example.sh
```
