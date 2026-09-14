# GSeg3D → Ground Consistency → Nav2 地形导航方案

> 适用场景：ROS 2 + 3D LiDAR + IMU + 已知 2D 栅格地图 + Nav2，机器人为轮式机器人或四足机器人。  
> 本文重点针对一个常见问题：**3D 激光定位存在 Z 轴漂移时，固定高度裁剪会把真实地面/斜坡误判为障碍物。**

---

## 1. 方案目标

当前已有系统通常类似：

```text
已知 2D Map
    |
    v
   Nav2
    |
    +----------------------+
                           |
3D LiDAR -> 按固定 Z 裁剪 -> ObstacleLayer / VoxelLayer
                           |
                           v
                      Local Costmap
```

例如使用下面的逻辑：

```cpp
if (point.z > min_obstacle_height &&
    point.z < max_obstacle_height)
{
    mark_as_obstacle(point);
}
```

这种方案的问题是，它依赖 **绝对 Z 高度**。

假设真实地面在：

```text
z = 0.0 m
```

而定位发生了：

```text
+0.30 m Z 漂移
```

那么整片地面可能变成：

```text
z = 0.30 m
```

如果障碍物检测阈值是：

```text
min_obstacle_height = 0.20 m
```

地面就可能被直接写成障碍物。

本文方案改成：

```text
3D LiDAR
    |
    v
  GSeg3D
    |
    +-------------------------+
    |                         |
ground_points           obstacle_points
    |                         |
    +------------+------------+
                 |
                 v
       Ground Consistency Layer
                 |
                 v
          Nav2 Local Costmap
                 |
                 v
             Controller
                 |
                 v
          四足机器人 / UGV
```

核心思想不是：

```text
“这个点的绝对 Z 是多少？”
```

而是：

```text
“这个点是不是地面？”

以及：

“这个障碍物相对附近地面高多少？”
```

因此，这套架构对整体 Z 偏移更加鲁棒。

---

# 2. 推荐系统架构

推荐将“全局导航”和“局部地形感知”分开。

```text
                           ┌──────────────────────┐
                           │   已知 2D 栅格地图   │
                           │ map.yaml + map.pgm   │
                           └──────────┬───────────┘
                                      │
                                      v
                              Nav2 Static Layer
                                      │
                                      v
                              Global Costmap
                                      │
                                      v
                             Global Planner
                            NavFn / Smac2D
                                      │
                                      │ global path
                                      v
+------------------------------------------------------------------+
|                         Local Perception                         |
|                                                                  |
|  3D LiDAR --------------------------+                            |
|                                     |                            |
|                                     v                            |
|                                  GSeg3D                          |
|                              /             \                     |
|                             /               \                    |
|                    ground_points      obstacle_points            |
|                             \               /                    |
|                              \             /                     |
|                               v           v                      |
|                         Ground Consistency                       |
|                                Layer                             |
|                                  |                               |
|                                  v                               |
|                           Local Costmap                          |
|                                                                  |
+------------------------------------------------------------------+
                                      |
                                      v
                              Nav2 Controller
                               MPPI / RPP
                                      |
                                      v
                                  cmd_vel
                                      |
                                      v
                        Quadruped locomotion controller
                                      |
                                      v
                             joint commands
```

对于四足机器人，要注意：

```text
Nav2 ≠ 足步规划器
```

Nav2 主要输出机器人整体的：

```text
vx
vy
wz
```

或者沿路径运动的整体目标。

真正的：

```text
足端落点
步态
身体姿态
MPC / WBC / RL locomotion
```

仍然由机器狗自己的运动控制器完成。

---

# 3. 为什么 GSeg3D 适合这个问题

GSeg3D 是一个基于 3D LiDAR 的 ground segmentation 方法。

输入：

```text
sensor_msgs/msg/PointCloud2
```

可选输入：

```text
sensor_msgs/msg/Imu
```

输出：

```text
/ground_segmentation/ground_points
/ground_segmentation/obstacle_points
/ground_segmentation/raw_points
```

它不是简单使用：

```text
z > threshold
```

而是对局部点云结构做地面判断。

其中一个关键参数是：

```yaml
slopeThresholdDegrees
```

它控制多大坡度的局部平面仍然允许被认为是 ground。

例如：

```text
                    墙
                    |
                    |
                ____|
              /
            /
          /
_________/
```

如果斜坡是 15°～20°，并且：

```yaml
slopeThresholdDegrees: 25.0
```

那么斜坡有机会继续被识别为：

```text
ground
```

而墙壁会被识别为：

```text
non-ground
```

这比基于固定 Z 的方法更适合坡道。

---

# 4. 一个重要概念：Ground 不等于 Traversable

对于四足机器人必须区分：

```text
ground
```

和：

```text
traversable
```

例如：

```text
45° 山坡
```

在几何意义上它完全可能是：

```text
ground
```

但机器狗可能只能安全通过：

```text
<= 25°
```

因此：

```text
GSeg3D 判断：
“这是连续地面。”

≠

机器狗判断：
“我一定可以走。”
```

本文第一阶段方案的目标是：

```text
普通平地
普通斜坡
缓坡
小范围高低起伏
```

不要错误地变成障碍物。

对于下面这些复杂需求：

```text
坡度 cost
台阶高度
坑洞
碎石
roughness
footstep affordance
```

后续建议增加：

```text
Elevation Mapping
+
Traversability Analysis
```

而不是继续把所有能力塞进 GSeg3D。

---

# 5. 软件组件

建议使用：

```text
ROS 2 Humble / Jazzy
Nav2
GSeg3D
ground_segmentation_ros2
nav2_ground_consistency_costmap_plugin
```

截至本文编写时：

- `ground_segmentation_ros2` 明确支持 ROS 2 Humble / Jazzy。
- Nav2 官方 Ground Terrain Segmentation 教程当前以 ROS 2 Jazzy 为主要示例。
- Ground Consistency Layer 已列在 Nav2 Navigation Plugins 中。

相关仓库：

```text
GSeg3D Core:
https://github.com/dfki-ric/ground_segmentation

GSeg3D ROS 2 Wrapper:
https://github.com/dfki-ric/ground_segmentation_ros2

Ground Consistency Costmap Plugin:
https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin

Nav2 官方教程:
https://ros-navigation.github.io/mkdocs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_ground_consistency_layer/navigation2_with_ground_consistency_layer/
```

---

# 6. 工作空间建议

例如：

```text
~/terrain_nav_ws/
└── src/
    ├── ground_segmentation/
    ├── ground_segmentation_ros2/
    ├── nav2_ground_consistency_costmap_plugin/
    └── your_robot_navigation/
```

创建：

```bash
mkdir -p ~/terrain_nav_ws/src
cd ~/terrain_nav_ws/src
```

克隆：

```bash
git clone https://github.com/dfki-ric/ground_segmentation.git
git clone https://github.com/dfki-ric/ground_segmentation_ros2.git
git clone https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin.git
```

安装常用依赖：

```bash
sudo apt update

sudo apt install \
    cmake \
    libpcl-dev \
    libeigen3-dev \
    libgtest-dev \
    libnanoflann-dev \
    python3-rosdep
```

然后：

```bash
cd ~/terrain_nav_ws

source /opt/ros/$ROS_DISTRO/setup.bash

rosdep install \
    --from-paths src \
    --ignore-src \
    -r -y
```

编译：

```bash
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

加载：

```bash
source install/setup.bash
```

检查：

```bash
ros2 pkg list | grep ground_segmentation
ros2 pkg list | grep ground_consistency
```

---

# 7. TF 设计

推荐至少保证：

```text
map
 |
 v
odom
 |
 v
base_link
 |
 +------ lidar_link
 |
 +------ imu_link
```

例如：

```text
map
 └── odom
      └── base_link
           ├── lidar_link
           └── imu_link
```

如果机器狗已有 `base_footprint`：

```text
map
 |
odom
 |
base_footprint
 |
base_link
 |
lidar_link
```

Nav2 可以根据现有机器人 TF 设计使用：

```yaml
robot_base_frame: base_footprint
```

或者：

```yaml
robot_base_frame: base_link
```

但 GSeg3D 的 `robot_frame` 推荐使用真实机器人机体坐标系：

```yaml
robot_frame: base_link
```

---

# 8. 为什么 IMU 很重要

机器狗上坡时，机体会产生明显 pitch：

```text
平地：

        robot
       _______
------|_______|------


上坡：

             _______
           /|_______|
         /
       /
-----/
```

如果点云没有根据重力方向正确理解姿态，斜坡与机器人自身 pitch 很容易混在一起。

推荐开启：

```yaml
use_imu_orientation: true
```

这样 GSeg3D 可以利用 IMU orientation 做 gravity alignment。

因此四足机器人建议：

```text
LiDAR
  +
IMU
  |
  v
GSeg3D
```

而不是：

```text
只有 LiDAR
   |
   v
按 map Z 切点云
```

---

# 9. GSeg3D 配置

GSeg3D 官方 ROS 2 wrapper 默认参数文件结构类似：

```yaml
ground_segmentation:
  ros__parameters:

    robot_frame: "base_link"

    maxX: 100.0
    minX: -100.0

    maxY: 100.0
    minY: -100.0

    maxZ: 100.0
    minZ: -100.0

    downsample: false
    downsample_resolution: 0.1

    lidar_to_ground: -1.78

    transform_tolerance: 0.1

    cellSizeX: 1.0
    cellSizeY: 1.0
    cellSizeZ: 10.0

    cellSizeZPhase2: 1.0

    slopeThresholdDegrees: 20.0

    groundInlierThreshold: 0.2

    centroidSearchRadius: 5.0

    use_imu_orientation: false

    maxGroundHeightDeviation: 0.1

    show_benchmark: true
```

这些值来自官方示例，不应直接照搬到你的机器狗。

---

# 10. 机器狗建议初始 GSeg3D 参数

假设目标是：

```text
平地
+
普通道路斜坡
+
20° 左右坡道
+
3D LiDAR
+
IMU
```

可以从下面开始：

```yaml
ground_segmentation:
  ros__parameters:

    robot_frame: "base_link"

    # ROI
    maxX: 20.0
    minX: -10.0

    maxY: 15.0
    minY: -15.0

    # 这里只做非常宽松的异常值裁剪
    # 不要再把它当成障碍物高度阈值。
    maxZ: 5.0
    minZ: -5.0

    # 根据点云密度决定
    downsample: true
    downsample_resolution: 0.08

    # 这里必须根据你的真实 LiDAR 安装高度填写。
    #
    # 官方定义：
    # LiDAR 到地面的垂直距离，
    # 地面在传感器下方时使用负值。
    #
    # 示例：雷达距地面约 0.45 m
    lidar_to_ground: -0.45

    transform_tolerance: 0.10

    # Phase I
    cellSizeX: 0.5
    cellSizeY: 0.5
    cellSizeZ: 5.0

    # Phase II
    cellSizeZPhase2: 0.5

    # 机器狗普通斜坡场景建议先从 25° 开始。
    #
    # 注意：
    # 这个参数代表“允许识别成 ground 的最大坡度”，
    # 不应该等同于机器狗真正的安全运动极限。
    slopeThresholdDegrees: 25.0

    # 越小越容易分离小障碍，
    # 但地面噪声大时容易碎。
    groundInlierThreshold: 0.10

    centroidSearchRadius: 2.5

    # 四足机器人建议打开
    use_imu_orientation: true

    # Phase II 地面高度连续性
    maxGroundHeightDeviation: 0.15

    show_benchmark: false
```

注意：

```text
上面不是“最终最优参数”
```

而是：

```text
第一轮实机测试起点
```

---

# 11. slopeThresholdDegrees 如何设置

不要这样设置：

```text
机器狗极限坡度 = 30°

所以：

slopeThresholdDegrees = 30°
```

更合理的思想是把：

```text
感知识别能力
```

和：

```text
运动能力
```

分开。

例如机器狗安全坡度：

```text
25°
```

可以先允许 GSeg3D 识别：

```yaml
slopeThresholdDegrees: 30.0
```

意思是：

```text
<= 30°：
仍可以认为这是一块连续地面
```

然后后续 traversability 层再做：

```text
0 ~ 15°   -> 低代价
15 ~ 20°  -> 中代价
20 ~ 25°  -> 高代价
>25°      -> 禁止
```

本文第一阶段还没有加入坡度代价层，所以为了安全，第一次实机建议先使用：

```yaml
slopeThresholdDegrees: 20.0
```

确认稳定之后逐步增加：

```text
20°
22°
25°
28°
...
```

不要一次调到非常大。

---

# 12. 启动 GSeg3D

官方 wrapper 的启动方式类似：

```bash
ros2 launch ground_segmentation_ros2 \
    ground_segmentation.launch.py \
    pointcloud_topic:=/your_lidar/points \
    imu_topic:=/your_imu/data \
    use_sim_time:=false
```

替换：

```text
/your_lidar/points
```

例如：

```text
/livox/lidar
/ouster/points
/velodyne_points
```

替换：

```text
/your_imu/data
```

为真实 IMU topic。

---

# 13. 首先不要启动 Nav2

第一次测试时只启动：

```text
LiDAR driver
IMU driver
TF
GSeg3D
RViz2
```

检查：

```bash
ros2 topic list | grep ground_segmentation
```

应该至少看到：

```text
/ground_segmentation/ground_points
/ground_segmentation/obstacle_points
/ground_segmentation/raw_points
```

检查频率：

```bash
ros2 topic hz /ground_segmentation/ground_points
```

```bash
ros2 topic hz /ground_segmentation/obstacle_points
```

---

# 14. RViz 调试

RViz 添加：

```text
PointCloud2
```

分别显示：

```text
/ground_segmentation/ground_points
```

和：

```text
/ground_segmentation/obstacle_points
```

建议用不同颜色。

目标效果：

```text
                墙
                M
                M
                M
            ____M
          G
        G
      G
GGGGGG
```

其中：

```text
G = ground
M = obstacle / non-ground
```

斜坡应该大部分属于：

```text
ground_points
```

墙、人、箱子等应该属于：

```text
obstacle_points
```

---

# 15. Ground Consistency Layer 的作用

GSeg3D 输出：

```text
ground_points
obstacle_points
```

Ground Consistency 不会简单执行：

```text
obstacle_points -> LETHAL
```

它会维护：

```text
ground evidence
+
nonground evidence
```

并结合局部地面高度判断障碍物是不是：

```text
真正阻挡机器人
```

核心可以理解为：

```text
height_relative =
    obstacle_z - local_ground_z
```

而不是：

```text
height_absolute =
    obstacle_z
```

这正是解决 Z 漂移问题的重要原因。

---

# 16. Z 漂移示例

正常时：

```text
ground_z   = 0.00
obstacle_z = 0.50
```

相对高度：

```text
0.50 - 0.00 = 0.50 m
```

如果定位整体产生：

```text
+0.30 m
```

那么：

```text
ground_z   = 0.30
obstacle_z = 0.80
```

相对高度仍然：

```text
0.80 - 0.30 = 0.50 m
```

因此比：

```text
if z > 0.2:
    obstacle
```

稳定得多。

注意：

> Ground Consistency 并不能修复定位系统本身。  
> 如果 TF 在短时间内剧烈跳变、roll/pitch 错误、时间同步严重错误，依然可能出现 costmap 异常。

---

# 17. Nav2 Local Costmap 推荐配置

核心建议：

```text
Local Costmap:
Ground Consistency
+
Inflation
```

第一版不要让 raw point cloud 同时进入：

```text
VoxelLayer
ObstacleLayer
```

否则容易出现：

```text
GSeg3D:
地面 -> FREE

Ground Consistency:
地面 -> FREE

VoxelLayer:
“这里有点” -> OCCUPIED

Master Costmap:
OBSTACLE
```

推荐：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:

      update_frequency: 10.0
      publish_frequency: 5.0

      global_frame: odom
      robot_base_frame: base_link

      rolling_window: true

      width: 8.0
      height: 8.0

      resolution: 0.05

      footprint: "[[0.35, 0.22],
                   [0.35, -0.22],
                   [-0.35, -0.22],
                   [-0.35, 0.22]]"

      plugins:
        - "ground_consistency"
        - "inflation_layer"

      ground_consistency:
        plugin: "nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer"

        ground_points_topic: "/ground_segmentation/ground_points"

        nonground_points_topic: "/ground_segmentation/obstacle_points"

        tf_timeout: 0.10

        # --------------------------------------------------
        # Robot dimensions
        # --------------------------------------------------

        # 改成机器狗实际导航碰撞高度
        robot_height: 0.65

        # 小于该相对高度的起伏可以忽略。
        #
        # 第一次实机建议保守一点：
        # 0.05 ~ 0.08 m
        min_clearance: 0.06

        # 高于该绝对过滤高度的输入点忽略。
        # 根据 LiDAR 安装和环境设置。
        maximum_height_filter: 2.0

        # --------------------------------------------------
        # Evidence
        # --------------------------------------------------

        ground_inc: 1.0
        nonground_inc: 1.5

        ground_decay: 0.80
        nonground_decay: 0.93

        max_score: 5000.0

        # 官方仓库默认是 2.0。
        # Nav2 教程示例使用过更高阈值。
        # 实机第一版可以从 4.0 开始，
        # 再根据误报/漏报调整。
        nonground_occ_thresh: 4.0

        nonground_prob_thresh: 0.75

        # --------------------------------------------------
        # Sparse-ground handling
        # --------------------------------------------------

        # 地面点比较密可以先 0。
        # 如果经常出现某个 cell 没有 ground，
        # 尝试 1~3。
        ground_neighbor_search_cells: 1

        # --------------------------------------------------
        # Performance
        # --------------------------------------------------

        # Local costmap 通常不需要保存 50m 范围。
        max_data_range: 10.0

        discretize_costs: true

        footprint_clearing_enabled: true

        enable_kpi_logging: false

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"

        inflation_radius: 0.35
        cost_scaling_factor: 3.0
```

注意：

```text
footprint
robot_height
min_clearance
inflation_radius
```

全部必须根据你的真实机器狗尺寸重新测量。

---

# 18. Global Costmap 建议

因为你已经有：

```text
已知 2D 栅格地图
```

Global Costmap 继续使用：

```text
Static Layer
+
Inflation Layer
```

例如：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:

      update_frequency: 2.0
      publish_frequency: 1.0

      global_frame: map
      robot_base_frame: base_link

      resolution: 0.05

      track_unknown_space: true

      footprint: "[[0.35, 0.22],
                   [0.35, -0.22],
                   [-0.35, -0.22],
                   [-0.35, 0.22]]"

      plugins:
        - "static_layer"
        - "inflation_layer"

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"

        map_subscribe_transient_local: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"

        inflation_radius: 0.35
        cost_scaling_factor: 3.0
```

此时：

```text
Global Planner
```

仍然在你的已知地图上规划。

实时 3D 地形信息主要影响：

```text
Local Costmap
```

---

# 19. 推荐 Planner / Controller

第一阶段没必要改变已有 Planner。

可以继续：

```text
NavFn
```

或者：

```text
SmacPlanner2D
```

对于四足机器人 local controller 可以优先考虑：

```text
MPPI Controller
```

如果机器狗底层只接受：

```text
vx
vy
wz
```

那么：

```text
Nav2 Controller
     |
     v
 geometry_msgs/Twist
     |
     v
 quadruped locomotion controller
```

即可。

如果机器狗支持全向运动：

```text
vx
vy
wz
```

则要保证 Nav2 的运动模型和 controller 参数允许 lateral motion。

---

# 20. 推荐启动顺序

不要一次启动整套系统然后看一堆错误。

建议按照下面顺序验证。

## Step 1：检查 TF

```bash
ros2 run tf2_tools view_frames
```

或者：

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

检查：

```text
base_link -> lidar_link
```

是否稳定。

检查：

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

---

## Step 2：检查 LiDAR

```bash
ros2 topic hz /your_lidar/points
```

RViz 看 raw cloud 是否正确。

---

## Step 3：检查 IMU

```bash
ros2 topic echo /your_imu/data --once
```

确认 orientation 不是全部：

```text
0 0 0 0
```

如果 IMU driver 不提供有效 orientation：

```yaml
use_imu_orientation: true
```

反而会出问题。

---

## Step 4：只启动 GSeg3D

检查：

```text
ground_points
obstacle_points
```

---

## Step 5：平地静止测试

目标：

```text
地板基本全部绿色 / ground
```

如果大量地板变成 obstacle：

优先检查：

```text
lidar_to_ground
TF
IMU orientation
groundInlierThreshold
```

---

## Step 6：斜坡测试

机器狗面对斜坡，但先不要走。

观察：

```text
斜坡是否属于 ground
```

如果坡面大量成为 obstacle：

逐步增加：

```yaml
slopeThresholdDegrees
```

例如：

```text
20
22
25
28
```

---

## Step 7：机器狗站到斜坡上

这是非常重要的一步。

测试：

```text
机体发生 pitch
+
LiDAR 姿态变化
```

之后 GSeg3D 是否仍然能识别：

```text
坡面 = ground
```

如果平地正常、机器人一上坡就错误：

重点检查：

```text
IMU orientation
LiDAR-IMU 外参
TF
时间同步
```

而不是首先去调：

```text
slopeThresholdDegrees
```

---

## Step 8：启动 Ground Consistency

此时先不要跑导航。

只看：

```text
local_costmap
```

---

## Step 9：人工移动机器狗

观察：

```text
平地
坡道
墙
箱子
小凸起
```

在 costmap 中如何变化。

---

## Step 10：最后启动 Nav2 Navigation

确认：

```text
Global Plan
```

正常后，再：

```text
Nav2 Goal
```

---

# 21. Ground Consistency 参数理解

## min_clearance

例如：

```yaml
min_clearance: 0.06
```

表示非常低的局部高度变化可以被认为不构成阻挡。

可以理解为：

```text
local ground
    |
    | 3 cm
    v
小凸起

-> FREE
```

而：

```text
local ground
    |
    | 20 cm
    v
箱子 / 台阶

-> OBSTACLE
```

不要一开始把它设置成机器狗理论上可以跨越的最大台阶高度。

机器狗“能够跨过去”并不代表：

```text
Nav2 2D collision model
```

就应该直接把它当作 free。

第一版建议保守。

---

# 22. nonground_occ_thresh

它控制：

```text
需要多少 non-ground evidence
```

之后才开始认为 cell 可能是障碍物。

过低：

```text
噪点
树叶
稀疏错误点
```

容易变成 obstacle。

过高：

```text
真实障碍物
```

需要较长时间才出现。

建议第一版：

```yaml
nonground_occ_thresh: 4.0
```

再根据实际表现修改。

---

# 23. nonground_prob_thresh

例如：

```yaml
nonground_prob_thresh: 0.75
```

要求 obstacle evidence 相对 ground evidence 足够占优。

如果：

```text
地面点很多
+
偶尔一个错误 obstacle 点
```

不会立即把整个 cell 标成 lethal。

---

# 24. ground_neighbor_search_cells

对于低线数 LiDAR、远距离点云或者坡面稀疏区域，一个 cell 可能只有 obstacle point，而没有对应 ground point。

例如：

```text
cell A:
ground = yes

cell B:
ground = no
obstacle = yes

cell C:
ground = yes
```

如果：

```yaml
ground_neighbor_search_cells: 1
```

Ground Consistency 可以利用邻近 cell 的地面高度估计 B。

对于稀疏点云可以尝试：

```text
1
2
3
```

但是越大：

```text
计算量增加
局部高度被过度平滑的风险增加
```

---

# 25. 不建议 raw cloud 再进入 VoxelLayer

错误结构：

```text
                      +-> GSeg3D
                      |     |
LiDAR raw cloud ------+     v
                      | Ground Consistency
                      |
                      +-> VoxelLayer
```

因为 raw cloud 中：

```text
地板
斜坡
```

本身也有大量 points。

VoxelLayer 并不知道：

```text
它们是可行走的 ground。
```

因此可能再次把地面压成二维障碍物。

---

# 26. 如果确实需要 VoxelLayer

例如必须检测：

```text
桌面
横杆
悬空障碍物
```

可以考虑：

```text
GSeg3D
   |
   +------ ground_points
   |
   +------ obstacle_points
              |
              v
          VoxelLayer
```

即：

```text
VoxelLayer 只吃 obstacle_points
```

不要让它再次处理：

```text
raw_points
```

但第一版本方案建议先完全不用 VoxelLayer，验证 Ground Consistency 能否满足需求。

---

# 27. 本方案如何降低 Z 漂移影响

原来的结构：

```text
map-frame point cloud
       |
absolute Z filter
       |
obstacle
```

强依赖：

```text
map Z
```

新结构：

```text
LiDAR
  |
robot-local segmentation
  |
ground / obstacle
  |
local-ground-relative height
  |
costmap
```

重点转移到了：

```text
局部几何关系
```

因此整体 Z 偏移的影响会明显下降。

但是：

```text
Z 漂移
```

和：

```text
roll / pitch 错误
```

不是一回事。

---

# 28. 对 roll / pitch 错误要更加警惕

假设真实地面：

```text
------------------------
```

如果姿态错误：

```text
////////////////////////
```

那么离机器人越远：

```text
Z 误差越大
```

这种问题仅仅依靠 Ground Consistency 很难完全解决。

因此优先保证：

```text
IMU
+
LiDAR-IMU 外参
+
时间同步
```

正确。

对机器狗来说，IMU gravity alignment 的重要程度通常高于绝对 Z。

---

# 29. 建议使用的 TF / 定位思想

对于 Nav2：

```text
map -> odom
```

负责全局定位关系。

```text
odom -> base_link
```

负责连续局部运动。

Nav2 主要真正关心：

```text
x
y
yaw
```

而 GSeg3D 感知更关心：

```text
局部 3D geometry
+
gravity direction
```

因此：

```text
Nav2 的 2D 导航
```

和：

```text
局部 3D 地形理解
```

可以解耦。

不应该再让：

```text
LIO 的绝对 Z
```

直接决定：

```text
这个点是不是障碍物
```

---

# 30. 机器狗斜坡第一轮测试建议

建议准备：

```text
0° 平地
10° 坡
15° 坡
20° 坡
25° 坡
```

每个坡分别测试：

```text
A. 机器狗在坡下
B. 机器狗前脚接触坡
C. 机器狗身体一半在坡
D. 机器狗完全站在坡上
E. 机器狗从坡顶向下
```

记录：

```text
ground point ratio
obstacle point ratio
local costmap lethal cells
TF errors
IMU orientation
CPU usage
```

这样比直接让机器人自主跑坡更容易定位问题。

---

# 31. 推荐调参顺序

不要同时调所有参数。

推荐：

```text
1. TF
2. lidar_to_ground
3. IMU orientation
4. slopeThresholdDegrees
5. groundInlierThreshold
6. cellSizeX / Y
7. cellSizeZPhase2
8. maxGroundHeightDeviation
9. Ground Consistency min_clearance
10. Evidence parameters
11. Inflation
12. Controller
```

---

# 32. GSeg3D 常见问题

## 问题 1：平地大量变成 obstacle

检查：

```text
lidar_to_ground
robot_frame
LiDAR TF
IMU orientation
groundInlierThreshold
```

---

## 问题 2：平地正常，斜坡变 obstacle

优先：

```yaml
slopeThresholdDegrees
```

逐渐增加。

同时检查：

```text
IMU
```

---

## 问题 3：小障碍被吃成 ground

尝试减小：

```yaml
groundInlierThreshold
```

也可以减小：

```yaml
cellSizeZPhase2
```

---

## 问题 4：粗糙地面 ground 很碎

可以稍微增加：

```yaml
groundInlierThreshold
```

或者适当增大：

```yaml
cellSizeX
cellSizeY
```

但是会降低细节。

---

# 33. Ground Consistency 常见问题

## Costmap 障碍物太多

检查：

```text
GSeg3D obstacle_points 是否已经错误
```

如果 GSeg3D 正常，再考虑：

```text
增大 nonground_occ_thresh
增大 nonground_prob_thresh
适当增加 ground_neighbor_search_cells
```

---

## Costmap 障碍物太少

可以：

```text
增大 nonground_inc
降低 nonground_occ_thresh
降低 nonground_prob_thresh
```

但安全系统不要一次改得过激进。

---

## 坡面局部突然出现 lethal cell

常见原因：

```text
某些 cell 缺少 ground point
```

可以尝试：

```yaml
ground_neighbor_search_cells: 1
```

之后：

```text
2
3
```

观察改善。

---

# 34. 推荐第一版完整数据流

```text
               /imu/data
                   |
                   |
/lidar/points      |
     |             |
     +------+------+
            |
            v
   ground_segmentation_ros2
            |
      +-----+------+
      |            |
      v            v
ground_points   obstacle_points
      |            |
      +-----+------+
            |
            v
nav2_ground_consistency_costmap_plugin
            |
            v
      Nav2 Local Costmap
            |
            +
            |
      Inflation Layer
            |
            v
      Nav2 Controller
            |
            v
          cmd_vel
            |
            v
quadruped locomotion controller
```

全局：

```text
map.yaml
   |
   v
Map Server
   |
   v
Static Layer
   |
   v
Global Costmap
   |
   v
Global Planner
   |
   v
global_path
```

最终：

```text
global_path
+
local terrain costmap
        |
        v
   Nav2 Controller
```

---

# 35. 推荐项目目录

可以建立自己的包：

```text
your_robot_terrain_nav/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── gseg3d.yaml
│   ├── nav2_params.yaml
│   └── localization.yaml
├── launch/
│   ├── terrain_perception.launch.py
│   ├── navigation.launch.py
│   └── bringup.launch.py
├── maps/
│   ├── map.yaml
│   └── map.pgm
└── rviz/
    └── terrain_nav.rviz
```

---

# 36. terrain_perception.launch.py 的职责

建议只负责：

```text
GSeg3D
```

以及必要的 topic remap。

例如逻辑上：

```text
/lidar/points
      |
      v
ground_segmentation_ros2
      |
      +--> /ground_segmentation/ground_points
      |
      +--> /ground_segmentation/obstacle_points
```

这样 perception 可以单独调试。

---

# 37. navigation.launch.py 的职责

负责：

```text
map_server
localization
Nav2
```

Ground Consistency Layer 由：

```text
nav2_params.yaml
```

配置。

---

# 38. bringup.launch.py 的职责

最终再组合：

```text
LiDAR / IMU
+
Localization
+
Terrain Perception
+
Nav2
```

不要一开始就把所有东西写进一个巨大 launch 文件。

---

# 39. 第一阶段验收条件

在认为方案可用之前，至少满足：

### 平地

```text
ground segmentation 稳定
costmap 不持续出现随机 lethal cell
```

### 坡道

```text
机器人在坡下：
坡面是 ground

机器人半上坡：
坡面仍是 ground

机器人完全站坡上：
坡面仍是 ground
```

### 障碍物

```text
箱子
人腿
墙
柱子
```

在 Local Costmap 中稳定出现。

### Z 漂移

如果 LIO Z 缓慢变化：

```text
地板不应该因为绝对 Z 改变而整体成为障碍。
```

---

# 40. 本方案目前不能解决的事情

这套第一阶段方案不会真正计算：

```text
坡度 cost
roughness
step height
坑洞深度
footstep foothold
四足稳定裕度
```

因此它主要回答：

```text
“这是地面还是障碍？”

+
“这个 non-ground 相对附近 ground 是否真的会挡住机器人？”
```

而不是完整回答：

```text
“这个地方机器狗通过的风险是多少？”
```

---

# 41. 后续升级方向

等第一阶段稳定后，可以扩展：

```text
3D LiDAR
   |
   v
Elevation Mapping
   |
   +------ elevation
   +------ normal
   +------ slope
   +------ roughness
   +------ step
   |
   v
Traversability Map
   |
   v
Nav2 Custom Costmap Layer
```

最终实现：

```text
0 ~ 10°    cost 0~20
10 ~ 15°   cost 30~60
15 ~ 20°   cost 80~120
20 ~ 25°   cost 150~220
> 25°      LETHAL
```

这个阶段可以研究：

```text
elevation_mapping_cupy
grid_map
自定义 Nav2 Costmap Layer
```

---

# 42. 推荐实施路线

## Phase 1

```text
LiDAR
+
GSeg3D
```

只解决：

```text
ground / obstacle
```

---

## Phase 2

```text
LiDAR
+
IMU
+
GSeg3D
```

重点解决：

```text
上坡时 pitch 导致的错误分割
```

---

## Phase 3

```text
GSeg3D
+
Ground Consistency
```

只看 costmap，不进行自主运动。

---

## Phase 4

```text
Known Map
+
Nav2
+
Ground Consistency
```

平地自主导航。

---

## Phase 5

```text
Known Map
+
Nav2
+
Ground Consistency
+
Quadruped Controller
```

缓坡自主导航。

---

## Phase 6

增加：

```text
Elevation Mapping
+
Traversability
```

实现真正 terrain-aware navigation。

---

# 43. 最终推荐

对于当前目标：

> 已知 2D 地图导航，并允许机器狗经过普通斜坡，同时避免由于 LiDAR/LIO 的 Z 漂移把地板当作障碍物。

第一版推荐：

```text
                  Known 2D Map
                       |
                       v
                 Nav2 Global
                       |
                       v
                  Global Path
                       |
                       |
3D LiDAR -----> GSeg3D
                  |
         +--------+--------+
         |                 |
         v                 v
 ground_points      obstacle_points
         |                 |
         +--------+--------+
                  |
                  v
         Ground Consistency
                  |
                  v
           Local Costmap
                  |
                  v
          Nav2 Controller
                  |
                  v
               cmd_vel
                  |
                  v
       Quadruped Locomotion
```

核心原则：

```text
不要再使用绝对 Z 高度
直接判断地面是不是障碍物。
```

应该尽量使用：

```text
局部 ground segmentation
+
相对 local ground 的障碍物高度
+
IMU gravity alignment
```

对于你之前遇到的：

```text
Z 漂移
→ 地板进入高度阈值
→ 地板变成障碍物
```

这是目前更合理的解决方向。

---

# 44. 参考资料

GSeg3D Core：

https://github.com/dfki-ric/ground_segmentation

GSeg3D ROS 2：

https://github.com/dfki-ric/ground_segmentation_ros2

Nav2 Ground Consistency Costmap Plugin：

https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin

Nav2 Ground Terrain Segmentation Tutorial：

https://ros-navigation.github.io/mkdocs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_ground_consistency_layer/navigation2_with_ground_consistency_layer/

Nav2 Navigation Plugins：

https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/

---

# 45. 下一步需要根据实机确认的信息

真正落地到你的机器人时，下一步需要确定：

```text
1. ROS 2 版本
   Humble / Jazzy

2. 3D LiDAR
   Livox Mid-360 / Ouster / Velodyne / RoboSense / 其他

3. LiDAR point cloud topic

4. IMU topic

5. LiDAR 相对 base_link 的 TF

6. LiDAR 离地高度

7. 机器狗尺寸
   length
   width
   body height

8. 机器狗安全最大坡度

9. 机器狗可接受的小台阶高度

10. 当前定位
    FAST-LIO
    FAST-LIO2
    Point-LIO
    LIO-SAM
    KISS-ICP
    其他

11. Nav2 当前 planner

12. Nav2 当前 controller

13. 现有 map->odom->base_link TF 的发布方式
```

拿到这些参数后，应该进一步生成：

```text
gseg3d.yaml
nav2_params.yaml
terrain_perception.launch.py
bringup.launch.py
TF/topic 数据流图
```

并直接按你的机器狗尺寸和传感器安装方式修改，而不是继续使用本文的示例数值。
