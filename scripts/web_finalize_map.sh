#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: web_finalize_map.sh MAP_PCD MAP_YAML" >&2
  exit 2
fi

map_pcd="$1"
map_yaml="$2"
mkdir -p "$(dirname -- "${map_pcd}")"

# FAST-LIO stays alive while this service snapshots the accumulated cloud.
timeout 35 ros2 service call /map_save std_srvs/srv/Trigger '{}'

if [ ! -s "${map_pcd}" ]; then
  echo "FAST-LIO did not produce ${map_pcd}" >&2
  exit 1
fi

ros2 run a2_terrain_nav pcd_to_nav2_map \
  --input "${map_pcd}" \
  --output "${map_yaml}"

test -s "${map_yaml}"
test -s "${map_yaml%.yaml}.pgm"
