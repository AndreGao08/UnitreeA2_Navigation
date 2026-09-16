#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
venv_dir="${project_root}/.venv/a2_web"
web_root="${project_root}/src/navigation/web"
deps_dir="${web_root}/.python-deps"

if /usr/bin/python3 -c 'import ensurepip' >/dev/null 2>&1; then
  if [ ! -x "${venv_dir}/bin/python" ]; then
    /usr/bin/python3 -m venv --system-site-packages "${venv_dir}"
  fi
  "${venv_dir}/bin/python" -m pip install \
    --disable-pip-version-check \
    -r "${web_root}/requirements.txt"
  echo "A2 web environment ready: ${venv_dir}"
else
  # Some ROS installations omit python3-venv. Keep ROS' system Python and
  # install compatible wheels into a project-local dependency directory.
  mkdir -p "${deps_dir}"
  python3 -m pip install --target "${deps_dir}" --only-binary=:all: \
    --implementation cp --python-version 3.10 --platform manylinux2014_x86_64 \
    -r "${web_root}/requirements.txt" exceptiongroup
  echo "A2 web dependencies ready: ${deps_dir}"
fi
