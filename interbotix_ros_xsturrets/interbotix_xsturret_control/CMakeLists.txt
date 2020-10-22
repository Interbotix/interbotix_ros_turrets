cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xsturret_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interbotix_xs_sdk
  interbotix_xs_modules
  interbotix_xsturret_descriptions
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS interbotix_xs_sdk interbotix_xs_modules interbotix_xsturret_descriptions
)
