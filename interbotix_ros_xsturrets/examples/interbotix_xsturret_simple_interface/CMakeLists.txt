cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xsturret_simple_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interbotix_xs_msgs
  interbotix_xsturret_control
  joy
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 CATKIN_DEPENDS interbotix_xs_msgs interbotix_xsturret_control joy roscpp rospy sensor_msgs std_msgs std_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(xsturret_simple_interface_joy src/xsturret_simple_interface_joy.cpp)
target_link_libraries(xsturret_simple_interface_joy ${catkin_LIBRARIES})
add_dependencies(xsturret_simple_interface_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/xsturret_simple_interface_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
