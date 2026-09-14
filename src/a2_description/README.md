# A2 description with Hesai JT128

The base model and meshes are copied from the official Unitree
`unitree_ros/robots/a2_description` package. The A2 mesh already contains its
embedded LiDAR housings, so the integration reuses Unitree's massless
`front_lidar_link` and `rear_lidar_link` frames instead of adding another visual
or collision body. Gazebo's JT128 uses a geometry-free Hesai optical frame at
the existing front LiDAR housing, because Hesai's native Z-axis convention is
different from Unitree's `front_lidar_link` convention.

The optical-frame origin (`xyz=0.33767 0 0.08134`) is copied from the official
A2 front LiDAR position. Replace it with a measured transform if the hardware
calibration differs.
