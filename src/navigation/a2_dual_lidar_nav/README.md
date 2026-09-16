# A2 dual-LiDAR navigation filter

This package is intentionally outside the mapping and localization pipelines.
It subscribes to the front and rear `sensor_msgs/msg/PointCloud2` streams,
retains the forward 180-degree hemisphere of each LiDAR in that LiDAR's own
frame, transforms accepted XYZ points to `base_link`, and publishes both inputs
on `/a2/navigation/lidar_points`.

Defaults:

| Parameter | Value |
|---|---|
| `front_topic` | `/lidar_points` |
| `rear_topic` | `/lidar_points_2` |
| `output_topic` | `/a2/navigation/lidar_points` |
| `target_frame` | `base_link` |
| `front_fov_deg` | `180.0` |
| `rear_fov_deg` | `180.0` |

The TF tree must provide a transform from each incoming cloud's `frame_id` to
`base_link`. The two inputs are processed independently, so a delayed or absent
rear stream does not block front-LiDAR obstacle updates.
