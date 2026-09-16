# Navigation module

This module owns terrain perception, Nav2 planning/control and the operator web
console.

- `a2_dual_lidar_nav`: front/rear 180-degree FOV filtering and navigation-only
  point-cloud aggregation.
- `a2_terrain_nav`: GSeg3D, Ground Consistency and Nav2 integration.
- `ground_segmentation*` and `nav2_ground_consistency_costmap_plugin`: pinned
  source dependencies managed by `scripts/fetch_navigation_dependencies.sh`.
- `web`: browser UI and ROS 2 backend.
- `docs` and `patches`: navigation design notes and compatibility changes.

Navigation consumes the localization contract and sends commands through the
localization safety gate before they reach the platform driver.
