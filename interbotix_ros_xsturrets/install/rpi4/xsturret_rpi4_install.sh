#!/usr/bin/env bash

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version == "18.04" ]; then
  ROS_NAME="melodic"
elif [ $ubuntu_version == "20.04" ]; then
  ROS_NAME="noetic"
else
  echo -e "Unsupported Ubuntu verison: $ubuntu_version"
  echo -e "Interbotix Turret only works with 18.04 or 20.04 on the Raspberry Pi"
  exit 1
fi

read -p "What is your robot model? (ex. wxxmd): " ROBOT_MODEL
read -p "Run the Joystick ROS package at system boot? " resp
if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
  run_joy_at_boot=true
else
  run_joy_at_boot=false
fi

echo "Ubuntu $ubuntu_version detected. ROS-$ROS_NAME chosen for installation.";

echo -e "\e[1;33m ******************************************** \e[0m"
echo -e "\e[1;33m The installation may take around 15 Minutes! \e[0m"
echo -e "\e[1;33m ******************************************** \e[0m"
sleep 4
start_time="$(date -u +%s)"

# Update the system
sudo apt update && sudo apt -y upgrade
sudo apt -y autoremove

# Install some necessary core packages
sudo apt -y install curl

# Step 1: Install ROS
if [ $(dpkg-query -W -f='${Status}' ros-$ROS_NAME-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "Installing ROS..."
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo apt update
  sudo apt -y install ros-$ROS_NAME-desktop-full
  if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
  fi
  echo "source /opt/ros/$ROS_NAME/setup.bash" >> ~/.bashrc
  if [ $ROS_NAME != "noetic" ]; then
    sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
  else
    sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  fi
  sudo rosdep init
  rosdep update
else
  echo "ros-$ROS_NAME-desktop-full is already installed!"
fi
source /opt/ros/$ROS_NAME/setup.bash

# Step 2: Install apriltag ROS Wrapper
APRILTAG_WS=~/apriltag_ws
if [ ! -d "$APRILTAG_WS/src" ]; then
  echo "Installing Apriltag ROS Wrapper..."
  mkdir -p $APRILTAG_WS/src
  cd $APRILTAG_WS/src
  git clone https://github.com/AprilRobotics/apriltag.git
  git clone https://github.com/AprilRobotics/apriltag_ros.git
  cd $APRILTAG_WS
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make_isolated
  echo "source $APRILTAG_WS/devel_isolated/setup.bash" >> ~/.bashrc
else
  echo "Apriltag ROS Wrapper already installed!"
fi
source $APRILTAG_WS/devel_isolated/setup.bash

# Step 3: Install Turret packages
INTERBOTIX_WS=~/interbotix_ws
if [ ! -d "$INTERBOTIX_WS/src" ]; then
  echo "Installing ROS packages for the Interbotix Turret..."
  mkdir -p $INTERBOTIX_WS/src
  cd $INTERBOTIX_WS/src
  git clone https://github.com/Interbotix/interbotix_ros_core.git
  git clone https://github.com/Interbotix/interbotix_ros_turrets.git
  git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git
  cd interbotix_ros_turrets && git checkout $ROS_NAME && cd ..
  rm interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE
  rm interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE
  rm interbotix_ros_toolboxes/interbotix_rpi_toolbox/CATKIN_IGNORE
  cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
  sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  cd $INTERBOTIX_WS
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  echo "source $INTERBOTIX_WS/devel/setup.bash" >> ~/.bashrc
else
  echo "Interbotix X-Series Turret ROS packages already installed!"
fi
source $INTERBOTIX_WS/devel/setup.bash

# Step 4: Setup Environment Variables
if [ -z "$ROS_IP" ]; then
  echo "Setting up Environment Variables..."
  echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
  echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
  echo "Environment variables already set!"
fi

# Step 5: Add desktop shortcut
cd $INTERBOTIX_WS/src/interbotix_ros_turrets/interbotix_ros_xsturrets/install/rpi4/
sudo cp xsturret.desktop /usr/share/applications/
cp xsturret.desktop ~/Desktop
chmod +x ~/Desktop/xsturret.desktop

# Step 6: Configure 'run at startup' feature
if [ "$run_joy_at_boot" = true ]; then
  touch xsturret_rpi4_launch.sh
  echo -e "#! /bin/bash

# This script is called by the xsturret_rpi4_boot.service file when
# the Raspberry Pi boots. It just sources the ROS related workspaces
# and launches the xsturret_simple_interface launch file.

source /opt/ros/$ROS_NAME/setup.bash
source $INTERBOTIX_WS/devel/setup.bash

# if the xs_sdk node is already running...
if pgrep -x \"roslaunch\" > /dev/null; then
  echo \"Launching turret GUI node only\"
  rosrun interbotix_xsturret_simple_interface xsturret_simple_interface_gui __ns:=$ROBOT_MODEL
else
  echo \"Launching...\"
  roslaunch interbotix_xsturret_simple_interface xsturret_simple_interface.launch use_rviz:=false robot_model:=$ROBOT_MODEL
fi" > xsturret_rpi4_launch.sh

  chmod +x xsturret_rpi4_launch.sh
  sudo cp xsturret_rpi4_boot.service /lib/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable xsturret_rpi4_boot.service
  echo -e "hdmi_force_hotplug=1\nhdmi_group=2\nhdmi_mode=82" | sudo tee -a /boot/firmware/usercfg.txt > /dev/null
fi

end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"

echo "Installation complete, took $elapsed seconds in total"
echo "NOTE: Remember to reboot the computer before using the robot!"
