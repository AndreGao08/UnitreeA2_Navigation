# Driver module

This module owns the Unitree A2 platform description and the Hesai JT128 input
boundary. It contains both simulation adapters and the complete physical LiDAR
driver source.

## Packages and sources

- `a2_description`: A2 meshes, URDF and JT128/IMU mounting frames.
- `a2_gazebo`: Gazebo worlds and robot simulation assets.
- `hesai_jt128_sim`: simulation-to-ROS adapters that reproduce the real sensor
  topic and point-field contract.
- `a2_hesai_driver`: project-owned JT128 configuration and launch entry point.
- `HesaiLidar_ROS_2.0`: vendored official Hesai ROS driver v2.0.12 at commit
  `e7e112f0809f0eed5e3c81c55a1a0376474db234`.
- `HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0`: vendored official SDK at
  commit `9d5dc4fc4ade5be5f6a6ca00e71dd4050b054168`.

The upstream driver and SDK retain their original licenses. See the `LICENSE`
files in both vendored source trees.

## Physical JT128

Edit `a2_hesai_driver/config/jt128.yaml` for the LiDAR and host network, then:

```bash
./scripts/build_ros2.sh
source install/setup.bash
ros2 launch a2_hesai_driver jt128.launch.py
```

The default outputs are `/lidar_points` and `/lidar_imu`, matching the
FAST-LIO JT128 configuration. Do not launch `hesai_jt128_sim` at the same time
as the physical driver because both publish the same sensor topics.
