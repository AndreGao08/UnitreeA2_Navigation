#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dependency_dir="${workspace_dir}/navigation"
mkdir -p "${dependency_dir}"

clone_at_revision() {
  local url="$1"
  local directory="$2"
  local revision="$3"
  local target="${dependency_dir}/${directory}"

  if [[ ! -d "${target}/.git" ]]; then
    git clone "${url}" "${target}"
    git -C "${target}" checkout --detach "${revision}"
    return
  fi

  local current
  current="$(git -C "${target}" rev-parse HEAD)"
  if [[ "${current}" != "${revision}" ]]; then
    echo "${directory} already exists at ${current}."
    echo "Expected ${revision}; preserving the existing checkout and any local work."
  fi
}

apply_patch_once() {
  local directory="$1"
  local patch_file="$2"
  local target="${dependency_dir}/${directory}"

  if git -C "${target}" apply --unidiff-zero --reverse --check "${patch_file}" >/dev/null 2>&1; then
    return
  fi
  if ! git -C "${target}" apply --unidiff-zero --check "${patch_file}"; then
    echo "Cannot apply ${patch_file}; preserving the existing checkout for inspection." >&2
    return 1
  fi
  git -C "${target}" apply --unidiff-zero "${patch_file}"
}

clone_at_revision \
  https://github.com/dfki-ric/ground_segmentation.git \
  ground_segmentation \
  e5aa4c2c47a961eae491056cf616fd415ff3d602

clone_at_revision \
  https://github.com/dfki-ric/ground_segmentation_ros2.git \
  ground_segmentation_ros2 \
  c0d60fd8ddcf561d006a907e04c84f9a8847372a

clone_at_revision \
  https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin.git \
  nav2_ground_consistency_costmap_plugin \
  41cec620efba6c370dccfc59a6ec1134775ff48a

apply_patch_once \
  nav2_ground_consistency_costmap_plugin \
  "${workspace_dir}/navigation/patches/nav2_ground_consistency_humble_tests.patch"

echo "Navigation source dependencies are ready under ${dependency_dir}."
