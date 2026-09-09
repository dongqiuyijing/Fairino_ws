cmake_minimum_required(VERSION 3.8)

project(fr3_part_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(
  part_rotate_demo
  src/part_rotate_demo.cpp
)


ament_target_dependencies(
  part_rotate_demo

  rclcpp

  moveit_ros_planning_interface
  moveit_msgs

  geometry_msgs
  shape_msgs

  tf2
  tf2_geometry_msgs
)


install(
  TARGETS
  part_rotate_demo

  DESTINATION
  lib/${PROJECT_NAME}
)


ament_package()