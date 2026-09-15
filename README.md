# Unitree A2 + Hesai JT128 localization (ROS 2 Humble)

This workspace integrates the official Unitree A2 model, a simulated 128-line
Hesai-compatible LiDAR and IMU, the ROS 2 branch of FAST-LIO2, standard TF
output, and trajectory evaluation in Gazebo Fortress.

## Phases 1-8 delivered

1. ROS 2 Humble workspace and dependency/build scripts.
2. Official A2 model and meshes packaged as `a2_description`.
3. Configurable JT128/IMU links, TF, collision geometry, and standing pose.
4. Gazebo Fortress sensors and JT128 PointCloud2/IMU adapters.
5. FAST-LIO ROS 2 integration with configurable frames, initialization window,
   map output, and corrected `odom -> base_link` output.
6. RViz, deterministic trajectories, ground-truth evaluation, unit tests, and
   synthetic/full-simulator headless validation scripts.
7. PCD map saving, map loading, coarse/fine scan-to-map relocalization,
   `map -> odom -> base_link` TF fusion, and global base pose output.
8. PCD-to-Nav2 map projection, GSeg3D ground segmentation, Ground Consistency
   local costmap, static global planning, and Nav2 `/cmd_vel` gait control.

## Packages

- `FAST_LIO_Hesai`: Hesai-adapted FAST-LIO2, checked out on its `ROS2` branch.
- `a2_description`: official A2 URDF/meshes plus configurable JT128 and IMU links.
- `a2_gazebo`: an asymmetric localization test world.
- `hesai_jt128_sim`: produces `x,y,z,intensity,ring,timestamp` PointCloud2 fields.
- `a2_localization_bringup`: one-command launch, TF adaptation, test trajectories,
  RViz, and APE-like ground-truth evaluation.
- `a2_map_localization`: PCD map publisher and ROS 2 scan-to-map relocalizer.
- `a2_terrain_nav`: GSeg3D/Ground Consistency/Nav2 configuration and launch.

## Install, build, run

```bash
./scripts/install_dependencies.sh
./scripts/fetch_navigation_dependencies.sh
./scripts/build_ros2.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py
```

To visualize the analytic A2 gait while keeping the commanded localization
trajectory accurate, use the assisted gait demonstration mode:

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  locomotion_mode:=gait_demo trajectory:=square
```

`gait_demo` animates all 12 joints and assists the base motion. Its floor is
visible to the simulated LiDAR but has no contact, so this mode is for FAST-LIO
and visualization rather than foot-contact validation. Use
`locomotion_mode:=dynamic` only for controller/contact experiments; the bundled
open-loop gait is not a speed-tracking A2 locomotion policy.

## Build and save a map

Run mapping in manual mode. The launch defaults to
`/home/gao/Documents/UnitreeA2_Localization/maps/a2_map.pcd` in this workspace:

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  operation_mode:=mapping \
  locomotion_mode:=gait_demo \
  trajectory:=manual
```

Send any `geometry_msgs/msg/Twist` command source to `/cmd_vel`, for example:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.15}}"
```

After covering the environment, stop the command publisher and press `Ctrl+C`
in the launch terminal. FAST-LIO automatically writes the PCD during clean
shutdown; no save service call is required. `/map_save` remains available only
as an optional mid-run snapshot. The output directory is created automatically.

## Relocalize in the saved map

Restart the simulation in relocalization mode with the same sensor extrinsics:

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  operation_mode:=relocalization \
  locomotion_mode:=gait_demo trajectory:=manual
```

Keep the robot still while FAST-LIO initializes, then use RViz **2D Pose
Estimate** to provide a rough `map -> base_link` pose. For repeatable simulation
started near the original map origin, add `auto_initialize:=true`.

The localizer performs coarse and fine ICP, rejects low-overlap or high-RMSE
matches, and then runs periodic low-frequency corrections. FAST-LIO remains the
continuous high-frequency odometry source. Useful interfaces are:

| Topic/service | Purpose |
|---|---|
| `/a2/map` | Saved PCD in the `map` frame |
| `/initialpose` | Rough `map -> base_link` initial estimate |
| `/a2/localization` | Relocalized `map -> base_link` odometry |
| `/a2/relocalization/status` | Initialization and match state |
| `/a2/relocalization/rmse` | Accepted/candidate registration RMSE |
| `/a2/relocalization/overlap` | Scan overlap fraction in `[0, 1]` |
| `/a2/relocalize` | Retry using the last accepted/initial estimate |
| `/a2/relocalization/reset` | Clear global localization and wait for a new initial pose |

This follows the low-rate global correction plus high-rate FAST-LIO approach in
[FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION),
ported from ROS 1/Python 2/Open3D to ROS 2 Humble/PCL. The initial pose is
properly converted from `map -> base_link` to `map -> odom`, and correction
updates are quality-gated, jump-limited, and smoothed.

The simulator launch defaults to `rmw_fastrtps_cpp` even if the parent shell
exports another RMW implementation. JT128 point clouds are large, and Fast DDS
uses its local shared-memory transport for the simulator, adapters, and
FAST-LIO. Override this only when deliberately testing another middleware:

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  rmw_implementation:=rmw_cyclonedds_cpp
```

For a repeatable headless test:

```bash
./scripts/test_ros2.sh
./scripts/run_synthetic_validation.sh
./scripts/run_headless_validation.sh 90
```

The synthetic validation does not require Gazebo; it verifies the exact JT128
point fields, LiDAR/IMU rates, FAST-LIO initialization, and `odom -> base_link`
output. The headless validation additionally runs the complete Gazebo trajectory
and writes error metrics under `/tmp/a2_localization_eval`.

The headless test defaults to a stationary convergence test. Select another
path with the optional third argument, for example
`./scripts/run_headless_validation.sh 90 /tmp/a2_eval figure8`.

The simulator reuses the official embedded front LiDAR housing; it does not add
a separate visible cylinder. The geometry-free Hesai optical frame is placed at
the official front LiDAR position, `xyz=[0.33767, 0, 0.08134] m`, with a Z-up
orientation for the JT128 scan. Override it if a measured hardware calibration
differs:

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  lidar_xyz:="0.33767 0.0 0.08134" lidar_rpy:="0.0 0.0 0.0"
```

The launch defaults to `manual`, so it only moves when a `/cmd_vel` source is
active. Scripted alternatives are `stationary`, `straight`, `square`, and
`figure8`.
Evaluation output defaults to `/tmp/a2_localization_eval`.

## ROS interfaces

| Topic | Type | Purpose |
|---|---|---|
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | FAST-LIO JT128 point cloud |
| `/lidar_imu` | `sensor_msgs/msg/Imu` | FAST-LIO SI-unit IMU |
| `/Odometry` | `nav_msgs/msg/Odometry` | Raw FAST-LIO IMU pose |
| `/a2/odometry` | `nav_msgs/msg/Odometry` | Corrected `odom -> base_link` pose |
| `/a2/localization` | `nav_msgs/msg/Odometry` | Relocalized `map -> base_link` pose |
| `/a2/map` | `sensor_msgs/msg/PointCloud2` | Loaded localization map |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Registered world cloud |
| `/a2/ground_truth/odom` | `nav_msgs/msg/Odometry` | Evaluation only |

The Gazebo ground truth is never fed into FAST-LIO.

Use `/a2/odometry` as the robot localization result. Its pose is
`T_odom_base_link = T_odom_imu * inverse(T_base_link_imu)`. The launch file uses
the same `lidar_xyz` and `lidar_rpy` values for both the URDF mount and this
conversion, so changing the JT128 mount does not silently change the reported
robot origin.

During relocalization, use `/a2/localization` as the global robot pose. The TF
chain is `map -> odom -> base_link`; `/a2/odometry` remains available as local
continuous odometry.

## Terrain-aware navigation

After saving `maps/a2_map.pcd`, launch the complete relocalization and navigation
stack with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py
```

For the bundled simulation map, automatic initialization is repeatable:

```bash
ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py auto_initialize:=true
```

Gazebo runs headlessly by default and RViz opens after about 10 seconds. This
reserves the rendering capacity needed by the GPU LiDAR. Add `gui:=true` when a
Gazebo window is required; the navigation default uses
`horizontal_samples:=128`, which can be raised on a faster GPU.

This combined launch defaults simulation, localization, RViz, and Nav2 to
`rmw_cyclonedds_cpp`; its runtime is installed by
`scripts/install_dependencies.sh`. If an external ROS 2 process sends goals or
inspects topics, run it with the same `RMW_IMPLEMENTATION` (sourcing this launch
does not alter the parent terminal).

The launch automatically creates or refreshes `maps/a2_nav2_map.yaml` and
`maps/a2_nav2_map.pgm` from the PCD. A cell must contain at least four obstacle
returns before it is marked occupied; this rejects sparse robot-body ghosts
left along the mapping trajectory. Override it only when necessary with
`minimum_points_per_cell:=N`. In RViz:

1. keep the robot stationary until FAST-LIO initializes;
2. use **2D Pose Estimate** to initialize `map -> odom`;
3. wait for `/a2/relocalization/status` to report localization success;
4. use **Nav2 Goal** to send a navigation target.

Nav2 publishes `/cmd_vel`; the existing A2 gait controller consumes it. The
global costmap uses the generated static map. The local costmap deliberately
does not consume the raw JT128 cloud through ObstacleLayer/VoxelLayer: GSeg3D
first separates ground and non-ground points, and Ground Consistency evaluates
non-ground height relative to nearby ground before inflation.

The navigation launch uses a dedicated 2 Hz scan-to-map correction profile so
the global `map -> base_link` feedback does not lag the assisted gait. The
robot's final global base pose is available on `/a2/localization` and through
the TF transform `map -> base_link`.

Useful diagnostics:

```bash
ros2 topic hz /ground_segmentation/ground_points
ros2 topic hz /ground_segmentation/obstacle_points
ros2 topic echo /a2/relocalization/status
ros2 lifecycle get /controller_server
ros2 action list | grep navigate
```

The bundled `gseg3d_a2.yaml` uses a nominal `lidar_to_ground: -0.46` for the
Gazebo gait demo. Measure and replace this value before using the stack on the
real robot. Ground classification is not the same as traversability; slope,
roughness, step and foothold cost layers remain a later upgrade.

For the current navigation profile, GSeg3D classifies only terrain below a
5-degree gravity-relative slope as traversable.  Terrain at or above 5 degrees
is sent to the local obstacle layer.

## Important real-robot boundary

The simulated LiDAR and IMU are collocated, so FAST-LIO uses identity extrinsics.
For hardware, replace the mount transform and FAST-LIO extrinsics with measured
or calibrated values and verify Hesai timestamp and IMU gyro units.

The default Gazebo horizontal resolution is 256 samples per ring so that the
128-line sensor can sustain its 10 Hz update rate on software rendering. Raise
`horizontal_samples` when GPU headroom permits it.
