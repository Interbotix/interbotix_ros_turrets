#! /bin/bash

# This script is called by the 'xsturret_rpi4_boot.service' file when
# the Raspberry Pi boots. It just sources the ROS related workspaces
# and launches the xsturret_simple_interface launch file.

source /opt/ros/melodic/setup.bash
source /home/pibot/interbotix_ws/devel/setup.bash

# if the xs_sdk node is already running...
if pgrep -x "roslaunch" > /dev/null
then
	echo "Launching turret GUI node only"
	rosrun interbotix_xsturret_simple_interface xsturret_simple_interface_gui __ns:=$ROBOT_MODEL
else
	echo "Launching..."
	roslaunch interbotix_xsturret_simple_interface xsturret_simple_interface.launch use_rviz:=false robot_model:=$ROBOT_MODEL
fi
