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
    src/localization/FAST_LIO_Hesai \
    src/localization/a2_localization_bringup \
    src/localization/a2_map_localization \
    src/navigation/a2_dual_lidar_nav \
    src/navigation/a2_terrain_nav \
    src/navigation/ground_segmentation \
    src/navigation/ground_segmentation_ros2 \
    src/navigation/nav2_ground_consistency_costmap_plugin \
    src/driver/a2_description \
    src/driver/a2_gazebo \
    src/driver/hesai_jt128_sim \
    src/driver/HesaiLidar_ROS_2.0 \
    src/driver/a2_hesai_driver \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
