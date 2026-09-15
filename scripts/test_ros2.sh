#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u
export PATH="/usr/bin:/bin:${PATH}"

cd "${workspace_dir}"
colcon test --packages-select \
  a2_dual_lidar_nav \
  hesai_jt128_sim \
  a2_localization_bringup \
  a2_terrain_nav \
  --base-paths \
  navigation/a2_dual_lidar_nav \
  driver/hesai_jt128_sim \
  localization/a2_localization_bringup \
  navigation/a2_terrain_nav \
  --event-handlers console_direct+
colcon test-result --verbose
