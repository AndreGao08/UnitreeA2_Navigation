# Localization module

This module owns LiDAR-inertial odometry, mapping and scan-to-map
relocalization.

- `FAST_LIO_Hesai`: Hesai-adapted ROS 2 FAST-LIO checkout.
- `a2_localization_bringup`: mapping/simulation launch and odometry/TF adapters.
- `a2_map_localization`: saved-map publication and global relocalization.
- `FAST_LIO_LOCALIZATION`: upstream reference checkout, not built directly.

Its public sensor boundary is `/lidar_points` plus `/lidar_imu`; its pose
boundary is `map -> odom -> base_link` and `/a2/localization`.
