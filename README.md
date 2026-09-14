# Unitree A2 + Hesai JT128 localization (ROS 2 Humble)

This workspace integrates the official Unitree A2 model, a simulated 128-line
Hesai-compatible LiDAR and IMU, the ROS 2 branch of FAST-LIO2, standard TF
output, and trajectory evaluation in Gazebo Fortress.

## Phases 1-7 delivered

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

## Packages

- `FAST_LIO_Hesai`: Hesai-adapted FAST-LIO2, checked out on its `ROS2` branch.
- `a2_description`: official A2 URDF/meshes plus configurable JT128 and IMU links.
- `a2_gazebo`: an asymmetric localization test world.
- `hesai_jt128_sim`: produces `x,y,z,intensity,ring,timestamp` PointCloud2 fields.
- `a2_localization_bringup`: one-command launch, TF adaptation, test trajectories,
  RViz, and APE-like ground-truth evaluation.
- `a2_map_localization`: PCD map publisher and ROS 2 scan-to-map relocalizer.

## Install, build, run

```bash
./scripts/install_dependencies.sh
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

## Important real-robot boundary

The simulated LiDAR and IMU are collocated, so FAST-LIO uses identity extrinsics.
For hardware, replace the mount transform and FAST-LIO extrinsics with measured
or calibrated values and verify Hesai timestamp and IMU gyro units.

The default Gazebo horizontal resolution is 256 samples per ring so that the
128-line sensor can sustain its 10 Hz update rate on software rendering. Raise
`horizontal_samples` when GPU headroom permits it.
