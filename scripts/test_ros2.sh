#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u
export PATH="/usr/bin:/bin:${PATH}"

cd "${workspace_dir}"
colcon test --packages-select hesai_jt128_sim a2_localization_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
