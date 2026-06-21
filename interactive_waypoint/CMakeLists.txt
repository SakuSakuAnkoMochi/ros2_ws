cmake_minimum_required(VERSION 3.8)

project(interactive_waypoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(interactive_markers REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

add_executable(
  waypoint_node

  src/main.cpp

  src/waypoint_node.cpp

  src/marker_manager.cpp

  src/yaml_io.cpp
)

ament_target_dependencies(

  waypoint_node

  rclcpp

  geometry_msgs

  visualization_msgs

  interactive_markers

  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(
  waypoint_node
  yaml-cpp
)

install(
  TARGETS
    waypoint_node

  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch

  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()