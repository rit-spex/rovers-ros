cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
    "msg/Can.msg"
    "msg/CanFD.msg"
    "msg/UDPPacket.msg"
    "msg/ArmWrist.msg"
    "msg/Pose.msg"
    "msg/Velocity.msg"
    "msg/PointList.msg"
    "msg/ArmCommand.msg"
    "msg/ArmPlannerStatus.msg"
    "msg/ArmTypingPlane.msg"
    "msg/ArmTypingStatus.msg"
    "msg/SpaceMouseAxes.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime geometry_msgs std_msgs)

# add_executable(custom_interfaces src/custom_interfaces.cpp)
# target_include_directories(custom_interfaces PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(custom_interfaces PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS custom_interfaces
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
