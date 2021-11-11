cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xsturret_object_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  camera_calibration
  cv_bridge
  interbotix_xsturret_control
  interbotix_xs_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  usb_cam
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS apriltag_ros camera_calibration cv_bridge interbotix_xsturret_control interbotix_xs_msgs rospy sensor_msgs std_msgs std_srvs usb_cam
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/xsturret_ar_tracker
  scripts/xsturret_color_tracker
  scripts/xsturret_color_tracker_calibrator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
