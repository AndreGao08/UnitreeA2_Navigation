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
    FAST_LIO_Hesai \
    src \
    third_party/ground_segmentation \
    third_party/ground_segmentation_ros2 \
    third_party/nav2_ground_consistency_costmap_plugin \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
