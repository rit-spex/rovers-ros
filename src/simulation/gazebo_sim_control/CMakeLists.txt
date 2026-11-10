cmake_minimum_required(VERSION 3.8)
project(gazebo_sim_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_sim_control_gui SHARED
  src/gazebo_sim_control_gui.cpp)
target_link_libraries(gazebo_sim_control_gui
  ${GAZEBO_LIBRARIES} Qt5::Widgets)

install(TARGETS gazebo_sim_control_gui
  LIBRARY DESTINATION lib)

ament_package()
