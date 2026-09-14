# Unitree A2 terrain navigation

This package combines the existing FAST-LIO/relocalization pipeline with:

- GSeg3D ground segmentation on `/lidar_points` and `/lidar_imu`;
- the Nav2 Ground Consistency local-costmap plugin;
- a static global costmap generated from the saved FAST-LIO PCD;
- NavFn global planning and Regulated Pure Pursuit control;
- the existing A2 gait controller through `/cmd_vel`.

The intended TF chain is `map -> odom -> base_link`. FAST-LIO supplies the
continuous `odom -> base_link` transform and `a2_map_localization` supplies the
low-rate global `map -> odom` correction after an RViz initial pose is given.

## Build

```bash
./scripts/fetch_navigation_dependencies.sh
./scripts/install_dependencies.sh
./scripts/build_ros2.sh
```

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py
```

Wait for FAST-LIO to initialize, set a rough pose with RViz **2D Pose
Estimate**, then use **Nav2 Goal**. The global planner uses `/map`; the local
controller uses `/ground_segmentation/ground_points` and
`/ground_segmentation/obstacle_points` through Ground Consistency.

The combined launch regenerates `maps/a2_nav2_map.yaml` and its PGM image when
`maps/a2_map.pcd` is newer. To convert explicitly:

```bash
ros2 run a2_terrain_nav pcd_to_nav2_map \
  --input maps/a2_map.pcd --output maps/a2_nav2_map.yaml
```

`lidar_to_ground` in `config/gseg3d_a2.yaml` is `-0.46 m` for the bundled
`gait_demo` model. Replace it with the measured sensor height on the real A2.
