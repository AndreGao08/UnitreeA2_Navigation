# A2 map localization

This ROS 2 Humble package loads a FAST-LIO PCD map and estimates `map -> odom`
from the current `/cloud_registered` scan. A rough `map -> base_link` pose comes
from RViz `/initialpose`; the node combines it with `/a2/odometry`, performs
coarse and fine PCL ICP, and publishes `/a2/localization` in the `map` frame.

Registration is accepted only when both the nearest-neighbour RMSE and overlap
threshold pass. Periodic corrections are bounded and smoothed. Configuration is
in `config/relocalization.yaml`.

For a real robot or rosbag where FAST-LIO and the A2 odometry adapter are
already running:

```bash
ros2 launch a2_map_localization relocalization.launch.py \
  map_path:=/home/gao/Documents/UnitreeA2_Localization/maps/a2_map.pcd
```
