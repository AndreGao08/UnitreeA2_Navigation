# Unitree A2 terrain navigation

This package combines the existing FAST-LIO/relocalization pipeline with:

- navigation-only 180-degree filtering of front `/lidar_points` and rear
  `/lidar_points_2` into `/a2/navigation/lidar_points`;
- GSeg3D ground segmentation on the filtered dual-LiDAR stream and `/lidar_imu`;
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

Add `auto_initialize:=true` for the bundled simulation map. Otherwise, wait for
FAST-LIO to initialize and set a rough pose with RViz **2D Pose Estimate**, then
use **Nav2 Goal**.

The navigation entry point runs Gazebo headlessly by default and starts RViz
after about 10 seconds, leaving graphics capacity for the simulated JT128. Use
`gui:=true` only when the Gazebo window is also needed. The stable simulation
default is 128 horizontal samples; override it with `horizontal_samples:=N`.

The combined navigation launch defaults all processes to CycloneDDS so RViz
goals and Nav2 action feedback share one reliable middleware configuration.
Override it with `rmw_implementation:=...` only when every participating ROS 2
process uses the same implementation.

The front and rear LiDAR topics can be changed without touching FAST-LIO:

```bash
ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py \
  front_lidar_topic:=/lidar_points \
  rear_lidar_topic:=/lidar_points_2
```

Each cloud is cropped to the forward 180-degree hemisphere in its own sensor
frame, transformed to `base_link`, and published independently on the shared
navigation topic. The rear sensor therefore augments navigation but never
enters mapping or relocalization. The robot TF tree must contain transforms
from both point-cloud frames to `base_link`.

The global planner uses `/map`; the local controller uses
`/ground_segmentation/ground_points` and
`/ground_segmentation/obstacle_points` through Ground Consistency.

The bundled GSeg3D profile treats only surfaces below the configured
`slopeThresholdDegrees: 5.0` limit as traversable ground.  It uses the IMU
gravity direction, so body pitch does not redefine a steep ramp as level
ground.  Surfaces at or above the limit are published on
`/ground_segmentation/obstacle_points` and become local-costmap obstacles.

The combined launch regenerates `maps/a2_nav2_map.yaml` and its PGM image when
`maps/a2_map.pcd` is newer. To convert explicitly:

```bash
ros2 run a2_terrain_nav pcd_to_nav2_map \
  --input maps/a2_map.pcd --output maps/a2_nav2_map.yaml
```

The projection requires four obstacle returns per cell by default, filtering
sparse body/leg points from the driven path. Tune this with
`minimum_points_per_cell:=N` at launch or `--minimum-points-per-cell N` in the
converter. The navigation-specific relocalization profile updates scan-to-map
corrections at 2 Hz. The requested global base pose is `/a2/localization`, with
the equivalent TF chain `map -> odom -> base_link`.

`lidar_to_ground` in `config/gseg3d_a2.yaml` is `-0.46 m` for the bundled
`gait_demo` model. Replace it with the measured sensor height on the real A2.
