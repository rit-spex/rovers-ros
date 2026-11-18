cmake_minimum_required(VERSION 3.8)
project(gazebo_sim_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# This package is a Gazebo Classic GUI plugin. Hardware-only machines (for
# example the main Jetson) do not have Gazebo's development files, but the
# full workspace still needs to configure and install cleanly. Build the
# plugin only when both Gazebo and Qt5 are present; otherwise leave an empty
# package behind so the rest of the build is unaffected.
find_package(gazebo QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets)

if(gazebo_FOUND AND Qt5_FOUND)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})

  add_library(gazebo_sim_control_gui SHARED
    src/gazebo_sim_control_gui.cpp)
  target_link_libraries(gazebo_sim_control_gui
    ${GAZEBO_LIBRARIES} Qt5::Widgets)

  install(TARGETS gazebo_sim_control_gui
    LIBRARY DESTINATION lib)
  message(STATUS "gazebo_sim_control: building Gazebo GUI plugin")
else()
  message(STATUS
    "gazebo_sim_control: Gazebo Classic or Qt5 not found; skipping GUI plugin")
endif()

ament_package()
