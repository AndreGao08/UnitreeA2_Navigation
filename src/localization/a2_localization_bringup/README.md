# Unitree A2 + JT128 ROS 2 localization bringup

This package brings up the A2 URDF, Hesai JT128 simulation, FAST-LIO, and the
`odom -> base_link` localization output on ROS 2 Humble with Gazebo Fortress.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Localization-only mode

The default mode keeps the A2 as a rigid sensor platform. It is useful for
repeatable FAST-LIO regression tests and does not simulate a legged controller.

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  locomotion_mode:=kinematic trajectory:=square
```

## A2 gait demonstration mode

`gait_demo` loads the 12-joint Gazebo controller and starts the ROS 2 A2 gait
node. It applies base velocity assistance so scripted paths and FAST-LIO tests
follow `/cmd_vel` while the legs are animated. World gravity is disabled in
this mode and the IMU adapter supplies the expected gravity specific force.
Its dedicated world keeps the floor visible to the simulated LiDAR but removes
floor contact; this prevents the URDF's initially straight legs from kicking
the assisted base before the joint controller reaches the standing pose.
This mode is intended for visualization and localization, not for evaluating
foot-contact dynamics.

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  locomotion_mode:=gait_demo trajectory:=square
```

For keyboard, joystick, or custom command testing, select manual mode and send
`geometry_msgs/msg/Twist` on `/cmd_vel`:

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  locomotion_mode:=gait_demo trajectory:=manual

ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.2}}"
```

Stop publishing and the 0.5 s watchdog ramps the command back to zero and
returns to the nominal stand. Controller state is available on
`/a2/locomotion/state` as `STAND_INITIALIZING`, `STAND`, or `TROT`. The gait
parameters are in `config/a2_gait.yaml`.

The included gait is a conservative analytic commissioning gait. Its joint
order, nominal pose, gains, effort limits, and gait period follow Unitree's A2
`unitree_rl_mjlab` deployment configuration, but it is not Unitree's proprietary
Sport controller and it is not a learned policy. The ROS-to-Gazebo contract is
a standard 12-joint `trajectory_msgs/msg/JointTrajectory`, so an exported A2
ONNX policy runner can replace the analytic target generator later. Unitree's
public repository provides A2 training and export code but does not include a
ready-to-run A2 policy weight in the checkout.

For controller research without base assistance, use
`locomotion_mode:=dynamic`. In that mode all translation must come from foot
contact. The included open-loop analytic gait is deliberately conservative and
does not accurately track the numerical velocity in `/cmd_vel`; use an exported
RL policy before treating it as a dynamics or controller benchmark.

## Pose and diagnostics

The requested robot body pose is published as:

- Topic: `/a2/odometry`
- Message: `nav_msgs/msg/Odometry`
- Parent frame: `odom`
- Child frame: `base_link`
- TF: `odom -> base_link`

Ground truth is available only for simulation evaluation on
`/a2/ground_truth/odom`. Joint feedback is `/joint_states`, and FAST-LIO input
health is `/a2_localization/input_ok`.

The launch defaults to `rmw_cyclonedds_cpp`, matching the navigation and Web
entry points. Use `rmw_implementation:=...` to override it consistently for
every participating ROS 2 process.
Evaluation CSV and JSON files are written to `/tmp/a2_localization_eval` by
default. Set `gazebo_verbosity:=4` only when detailed Gazebo controller logs are
needed.

## Mapping and relocalization

Start manual `/cmd_vel` mapping. The default output is the workspace file
`maps/a2_map.pcd`:

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  operation_mode:=mapping locomotion_mode:=gait_demo trajectory:=manual
```

Control the robot by publishing `geometry_msgs/msg/Twist` on `/cmd_vel`. After
covering the environment, press `Ctrl+C` in the launch terminal. A clean exit
automatically saves the map, so calling `/map_save` is unnecessary. That
service remains available only when an optional mid-run snapshot is wanted.

Load and relocalize against it in a new run:

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  operation_mode:=relocalization \
  locomotion_mode:=gait_demo trajectory:=manual
```

Set the rough pose with RViz **2D Pose Estimate** and wait until
`/a2/relocalization/status` reports `LOCALIZED`. The global body result is
`/a2/localization` (`map -> base_link`); local FAST-LIO output remains
`/a2/odometry` (`odom -> base_link`). Use `auto_initialize:=true` only when the
robot begins near the map's original origin.

Gazebo's GPU LiDAR publishes an instantaneous rendered snapshot, so its points
share one timestamp. Rolling-scan timestamps must remain disabled for this
source; otherwise FAST-LIO would deskew motion that is not present in the cloud.
