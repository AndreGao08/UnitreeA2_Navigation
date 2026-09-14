#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_seconds="${1:-90}"
output_dir="${2:-/tmp/a2_localization_eval}"
trajectory="${3:-stationary}"

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

missing_packages=()
for package in xacro ros_gz_sim ros_gz_bridge ros_gz_interfaces; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    missing_packages+=("${package}")
  fi
done
if (( ${#missing_packages[@]} > 0 )); then
  echo "Missing ROS 2 packages: ${missing_packages[*]}" >&2
  echo "Run ${workspace_dir}/scripts/install_dependencies.sh first." >&2
  exit 2
fi

set +e
timeout --foreground --signal=INT --kill-after=10s "${run_seconds}" \
  ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  gui:=false rviz:=false trajectory:="${trajectory}" eval_directory:="${output_dir}" \
  operation_mode:=mapping map_path:=/tmp/a2_fastlio_map.pcd
launch_status=$?
set -e

if [[ ${launch_status} -ne 0 && ${launch_status} -ne 124 && ${launch_status} -ne 130 ]]; then
  echo "Gazebo validation launch failed with status ${launch_status}" >&2
  exit "${launch_status}"
fi

summary="${output_dir}/summary.json"
if [[ ! -s "${summary}" ]]; then
  echo "No evaluation summary was produced at ${summary}" >&2
  exit 1
fi

/usr/bin/python3 - "${summary}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding='utf-8') as stream:
    result = json.load(stream)
print(json.dumps(result, indent=2))
if result.get('samples', 0) < 10:
    raise SystemExit('validation collected fewer than 10 synchronized samples')
PY
