#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
venv_python="${project_root}/.venv/a2_web/bin/python"
web_map_dir="${project_root}/maps/web/default"

if [ -x "${venv_python}" ] && "${venv_python}" -c 'import uvicorn' >/dev/null 2>&1; then
  web_python="${venv_python}"
else
  web_python="/usr/bin/python3.10"
  export PYTHONPATH="${project_root}/web/.python-deps${PYTHONPATH:+:${PYTHONPATH}}"
fi

if [ "${web_python}" = "/usr/bin/python3.10" ] && [ ! -d "${project_root}/web/.python-deps" ]; then
  echo "Web environment is missing; run: bash scripts/setup_web.sh" >&2
  exit 1
fi

mkdir -p "${web_map_dir}"
for extension in pcd pgm yaml; do
  source_map="${project_root}/maps/a2_$([ "${extension}" = pcd ] && echo map || echo nav2_map).${extension}"
  if [ -s "${source_map}" ] && [ ! -e "${web_map_dir}/a2_map.${extension}" ]; then
    cp -- "${source_map}" "${web_map_dir}/a2_map.${extension}"
  fi
done

# The copied YAML must refer to the renamed image in its own map directory.
if [ -f "${web_map_dir}/a2_map.yaml" ]; then
  sed -i 's#^image:.*#image: a2_map.pgm#' "${web_map_dir}/a2_map.yaml"
fi

set +u
source "${project_root}/scripts/web_ros_env.sh"
set -u
cd "${project_root}"
exec "${web_python}" -m uvicorn robot_server.app.main:app \
  --host "${A2_WEB_HOST:-0.0.0.0}" \
  --port "${A2_WEB_PORT:-8080}"
