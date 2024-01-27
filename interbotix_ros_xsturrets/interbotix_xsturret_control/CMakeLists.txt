cmake_minimum_required(VERSION 3.5)
project(interbotix_xsturret_control)

find_package(ament_cmake REQUIRED)

# Install Python executables
install(
  PROGRAMS
    demos/turret_mixed_profile_control.py
    demos/turret_time_profile_control.py
    demos/turret_velocity_profile_control.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
