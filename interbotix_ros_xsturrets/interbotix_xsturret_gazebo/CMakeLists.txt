cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xsturret_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  gazebo_ros
  gazebo_ros_control
  interbotix_xsturret_descriptions
  joint_state_controller
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS controller_manager effort_controllers gazebo_ros gazebo_ros_control interbotix_xsturret_descriptions joint_state_controller
)
