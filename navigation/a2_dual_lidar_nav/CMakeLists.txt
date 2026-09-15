cmake_minimum_required(VERSION 3.8)
project(a2_dual_lidar_nav)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(dual_lidar_nav_filter src/dual_lidar_nav_filter.cpp)
target_compile_features(dual_lidar_nav_filter PUBLIC cxx_std_17)
target_include_directories(dual_lidar_nav_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dual_lidar_nav_filter
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros)

install(TARGETS dual_lidar_nav_filter
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fov_filter test/test_fov_filter.cpp)
  target_compile_features(test_fov_filter PUBLIC cxx_std_17)
  target_include_directories(test_fov_filter PRIVATE include)
endif()

ament_package()
