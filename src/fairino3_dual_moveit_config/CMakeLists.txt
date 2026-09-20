cmake_minimum_required(VERSION 3.22)
project(fairino3_dual_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME})

ament_package()
