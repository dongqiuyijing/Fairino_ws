cmake_minimum_required(VERSION 3.8)
project(fairino_hardware_dual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fairino_msgs REQUIRED)

set(FR_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../frcobot_ros2/fairino_hardware_v3_9_7")
set(FR_SDK_LIB "${FR_SDK_ROOT}/libfairino/lib/libfairino.so.2.3.7")

add_library(fairino_dual_hardware SHARED
  src/fairino_dual_hardware_interface.cpp
)

target_include_directories(fairino_dual_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${FR_SDK_ROOT}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fairino_dual_hardware
  "-Wl,-rpath,\$ORIGIN/.."
  ${FR_SDK_LIB}
)

ament_target_dependencies(fairino_dual_hardware
  hardware_interface
  pluginlib
  rclcpp
  fairino_msgs
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface fairino_hardware_dual.xml)

install(TARGETS fairino_dual_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(fairino_dual_hardware)
ament_export_dependencies(
  ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle fairino_msgs)

ament_package()
