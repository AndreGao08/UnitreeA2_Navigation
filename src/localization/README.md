# Localization module

This module owns LiDAR-inertial odometry, mapping and scan-to-map
relocalization.

- `FAST_LIO_Hesai`: vendored Hesai-adapted ROS 2 FAST-LIO source, including
  the project-specific ROS 2 integration fixes and `ikd-Tree` source.
- `a2_localization_bringup`: mapping/simulation launch and odometry/TF adapters.
- `a2_map_localization`: saved-map publication and global relocalization.
- `FAST_LIO_LOCALIZATION`: vendored upstream reference source, including its
  `ikd-Tree` dependency; it is not built directly.

Its public sensor boundary is `/lidar_points` plus `/lidar_imu`; its pose
boundary is `map -> odom -> base_link` and `/a2/localization`.
