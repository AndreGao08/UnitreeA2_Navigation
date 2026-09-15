#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u

# A Conda Python on PATH can load an incompatible Empy version into ROSIDL.
# Put the Ubuntu Python first for every ROS 2 build.
export PATH="/usr/bin:/bin:${PATH}"

cd "${workspace_dir}"
colcon build \
  --base-paths \
    localization/FAST_LIO_Hesai \
    localization/a2_localization_bringup \
    localization/a2_map_localization \
    navigation/a2_dual_lidar_nav \
    navigation/a2_terrain_nav \
    navigation/ground_segmentation \
    navigation/ground_segmentation_ros2 \
    navigation/nav2_ground_consistency_costmap_plugin \
    driver/a2_description \
    driver/a2_gazebo \
    driver/hesai_jt128_sim \
    driver/HesaiLidar_ROS_2.0 \
    driver/a2_hesai_driver \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
