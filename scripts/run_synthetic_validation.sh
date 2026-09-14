#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
validation_dir="$(mktemp -d /tmp/a2_synthetic_validation.XXXXXX)"
launch_log="${validation_dir}/launch.log"
input_status="${validation_dir}/input_status.yaml"
odometry="${validation_dir}/odometry.yaml"

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

launch_pid=""
stop_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap stop_launch EXIT

setsid ros2 launch a2_localization_bringup synthetic_lio_smoke.launch.py \
  >"${launch_log}" 2>&1 &
launch_pid=$!

wait_for_topic() {
  local topic="$1"
  local attempts="${2:-80}"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      return 0
    fi
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      return 1
    fi
    sleep 0.25
  done
  return 1
}

if ! wait_for_topic /a2_localization/input_ok || \
   ! timeout 15 ros2 topic echo /a2_localization/input_ok std_msgs/msg/Bool \
     --once >"${input_status}"; then
  echo "Timed out waiting for the JT128 input monitor." >&2
  tail -n 80 "${launch_log}" >&2
  exit 1
fi
if ! grep -q 'data: true' "${input_status}"; then
  echo "JT128 input contract or sensor rates are invalid." >&2
  cat "${input_status}" >&2
  tail -n 80 "${launch_log}" >&2
  exit 1
fi

if ! wait_for_topic /a2/odometry 120 || \
   ! timeout 20 ros2 topic echo /a2/odometry nav_msgs/msg/Odometry --once >"${odometry}"; then
  echo "Timed out waiting for FAST-LIO odometry." >&2
  tail -n 120 "${launch_log}" >&2
  exit 1
fi
if ! grep -q 'frame_id: odom' "${odometry}" || \
   ! grep -q 'child_frame_id: base_link' "${odometry}"; then
  echo "FAST-LIO odometry uses an unexpected TF contract." >&2
  cat "${odometry}" >&2
  exit 1
fi

echo "Synthetic JT128 -> FAST-LIO validation passed."
echo "  input contract: OK"
echo "  odometry:       odom -> base_link"
echo "  artifacts:      ${validation_dir}"
