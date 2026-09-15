cmake_minimum_required(VERSION 3.8)
project(a2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf rviz
  DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE.unitree_ros
  DESTINATION share/${PROJECT_NAME})

ament_package()
