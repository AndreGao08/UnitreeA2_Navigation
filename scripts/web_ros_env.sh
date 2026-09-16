#!/usr/bin/env bash

# Shared ROS environment for the A2 web backend and all processes it starts.
_a2_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export A2_NAVIGATION_WS="$(cd -- "${_a2_script_dir}/.." && pwd)"
# Compatibility for existing user profiles and saved web configurations.
export A2_LOCALIZATION_WS="${A2_NAVIGATION_WS}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

source /opt/ros/humble/setup.bash
if [ ! -f "${A2_NAVIGATION_WS}/install/setup.bash" ]; then
  echo "A2 workspace is not built: ${A2_NAVIGATION_WS}/install/setup.bash" >&2
  return 1 2>/dev/null || exit 1
fi
source "${A2_NAVIGATION_WS}/install/setup.bash"

export PYTHONPATH="${A2_NAVIGATION_WS}/src/navigation/web${PYTHONPATH:+:${PYTHONPATH}}"
unset _a2_script_dir
