cmake_minimum_required(VERSION 3.5)
project(teensy_comms)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(teensy_comms SHARED
  src/teensy_comms.cpp)

rclcpp_components_register_node(teensy_comms
  PLUGIN "teensy_comms::LaserScanToPointCloudNode"
  EXECUTABLE teensy_comms_node)

ament_auto_add_library(teensy_comms SHARED
  src/teensy_comms_node.cpp)

ament_auto_add_executable(dummy_teensy_comms
  src/dummy_teensy_comms.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
