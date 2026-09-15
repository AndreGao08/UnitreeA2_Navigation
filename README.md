# UnitreeA2_Navigation

Unitree A2 + Hesai JT128 定位与导航工程（ROS 2 Humble）。工程同时支持
Gazebo Fortress 仿真输入和 JT128 真机驱动，使用 FAST-LIO、点云地图重定位、
GSeg3D 地形分割和 Nav2 完成建图、定位与导航。

## 一、总体坐标系和数据流

系统使用以下 TF 坐标链：

```text
map -> odom -> base_link
```

- FAST-LIO 输出高频局部里程计 `odom -> base_link`。
- `a2_map_localization` 使用保存的 PCD 地图，估计并修正低频的
  `map -> odom`。
- 机器人最终的全局机身位姿是 TF 中的 `map -> base_link`，同时发布在
  `/a2/localization`。
- `/a2/odometry` 是局部 `odom -> base_link` 位姿，不能直接当作全局地图位姿。
- `/a2/ground_truth/odom` 只用于仿真评估，不会输入 FAST-LIO 或导航。

完整数据和控制链路如下：

```text
前雷达 /lidar_points ── FAST-LIO ── /a2/odometry ── odom -> base_link
       │                    │
       │                 建图时保存 maps/a2_map.pcd
       │
       └─ 前向 180° ─┐
后雷达 /lidar_points_2 ─ 后向安装雷达自身前向 180° ─┤
                                                    └─ /a2/navigation/lidar_points
                                                               │
                                                    GSeg3D（结合 IMU 判断坡度）
                       │
          Nav2 静态全局代价地图 + Ground Consistency 局部代价地图
                       │
          /goal_pose（RViz/网页） -> Nav2 -> /cmd_vel_nav
                       │
              velocity_smoother -> /cmd_vel
                       │
       定位安全门 -> /a2/safe_cmd_vel -> A2 gait_controller -> Gazebo
```

## 二、模块划分

工程源码按职责分成三个顶层模块：

```text
UnitreeA2_Navigation/
├── localization/  # FAST-LIO、建图、TF 与地图重定位
├── navigation/    # GSeg3D、Nav2、网页控制台与导航依赖
└── driver/        # A2 描述/仿真、JT128 仿真适配与真机驱动
```

| 模块 | 主要内容 | 对外边界 |
|---|---|---|
| `localization/` | `FAST_LIO_Hesai`、`a2_localization_bringup`、`a2_map_localization` | 输入 `/lidar_points`、`/lidar_imu`；输出 `map -> odom -> base_link` |
| `navigation/` | `a2_dual_lidar_nav`、`a2_terrain_nav`、GSeg3D、Ground Consistency、`web` | 输入前后雷达、定位和地图；输出 `/cmd_vel_nav` |
| `driver/` | `a2_description`、`a2_gazebo`、`hesai_jt128_sim`、`a2_hesai_driver`、官方 `HesaiLidar_ROS_2.0` | 输出统一的 JT128 点云/IMU；接收安全速度命令 |

各模块的详细说明见 `localization/README.md`、`navigation/README.md` 和
`driver/README.md`。

## 三、安装和编译

在工程根目录执行：

```bash
source /opt/ros/humble/setup.bash
./scripts/install_dependencies.sh
./scripts/fetch_navigation_dependencies.sh
./scripts/build_ros2.sh
source install/setup.bash
```

`build_ros2.sh` 只构建三个模块中明确列出的 ROS 包，避免把参考仓库误当成
运行包。编译完成后加载环境：

```bash
source install/setup.bash
```

### 3.1 JT128 真机驱动

工程已内置 Hesai 官方 ROS 驱动 v2.0.12 及其 SDK。先修改
`driver/a2_hesai_driver/config/jt128.yaml` 中的雷达 IP、主机 IP 和端口，再运行：

```bash
ros2 launch a2_hesai_driver jt128.launch.py
```

真机驱动输出 `/lidar_points` 和 `/lidar_imu`，与 FAST-LIO 输入直接一致。
真机运行时不要同时启动 `hesai_jt128_sim`。

## 四、建图流程

### 4.1 启动建图仿真

建图模式默认使用工程内的 `maps/a2_map.pcd` 作为输出文件：

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  operation_mode:=mapping \
  locomotion_mode:=gait_demo \
  trajectory:=manual
```

`manual` 表示机器人不会自动运动，需要外部发布 `/cmd_vel`。例如：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25, y: 0.0}, angular: {z: 0.15}}"
```

应让机器人覆盖需要建图的区域，并尽量保持传感器运动连续。建图期间可在
RViz 中观察 `/cloud_registered`。

### 4.2 保存地图

覆盖环境后，先停止速度发布，再回到启动仿真终端按 `Ctrl+C`。FAST-LIO
在干净退出时会自动保存：

```text
maps/a2_map.pcd
```

工程也保留 `/map_save` 服务用于中途快照，但正常流程只需退出启动程序即可。
不要在 FAST-LIO 仍在写文件时强制杀进程。

## 五、定位和重定位流程

### 5.1 启动定位

使用已经保存的 PCD：

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  operation_mode:=relocalization \
  map_path:=maps/a2_map.pcd \
  locomotion_mode:=gait_demo \
  trajectory:=manual
```

启动后先让机器人保持静止，等待 FAST-LIO 完成 IMU 和雷达初始化。然后在
RViz 使用 **2D Pose Estimate** 发布 `/initialpose`，给出机器人在 `map`
坐标系中的大致位置和朝向。

对于从地图原点附近开始的可重复仿真，可以使用：

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  operation_mode:=relocalization \
  map_path:=maps/a2_map.pcd \
  auto_initialize:=true \
  locomotion_mode:=gait_demo \
  trajectory:=manual
```

### 5.2 重定位判定

重定位模块执行粗配准和精配准，并根据 RMSE、点云重叠率和位姿跳变阈值
拒绝错误匹配。通过后发布 `map -> odom` 修正。FAST-LIO 仍然负责高频
局部运动估计。

导航前必须同时满足：

- `/a2/relocalization/status` 为 `LOCALIZED`；
- `/a2/odometry/status` 为 `OK`；
- TF 中存在 `map -> odom -> base_link`；
- `/a2/odometry` 持续发布。

只要定位状态不安全，定位安全门就会发布零速度，机器人不会移动。

## 六、地形分割和导航逻辑

### 6.1 生成 Nav2 地图

导航启动时，将 `maps/a2_map.pcd` 投影成 Nav2 使用的：

```text
maps/a2_nav2_map.yaml
maps/a2_nav2_map.pgm
```

默认每个栅格至少需要 4 个障碍点才标记为占用，用于过滤机器人腿和机身
在行走过程中留下的稀疏点。必要时可调整：

```bash
ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py \
  minimum_points_per_cell:=N
```

### 6.2 GSeg3D 可通行判定

当前工程按用户要求将重力方向相对坡度 `<= 5°` 的区域视为可通行地面：

- 坡度小于等于 5°：发布到 `/ground_segmentation/ground_points`；
- 坡度大于 5°：发布到 `/ground_segmentation/obstacle_points`；
- 局部代价地图只把非地面点和其膨胀区域作为障碍。

这里的 5° 是相对于 IMU 重力方向计算的，不是相对于雷达坐标系或机器人
当前俯仰角计算的。当前逻辑只处理坡度，不包含台阶高度、粗糙度和足端落点
规划。

### 6.3 启动完整导航

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py \
  auto_initialize:=true
```

默认行为：

- Gazebo 后台运行，RViz 自动启动；
- 载入 PCD 并生成 Nav2 地图；
- 启动 FAST-LIO、重定位、GSeg3D、Nav2 和安全门；
- 使用 `gait_demo` 解析步态和机身辅助运动；
- 目标坐标系统一为 `map`。

需要 Gazebo 图形界面时：

```bash
ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py \
  auto_initialize:=true gui:=true
```

### 6.4 导航目标和速度链路

RViz 的 **Nav2 Goal** 和网页端都发布：

```text
/goal_pose  geometry_msgs/msg/PoseStamped
```

消息的 `header.frame_id` 必须是 `map`。Nav2 处理目标后依次经过：

```text
/cmd_vel_nav
    -> velocity_smoother
/cmd_vel
    -> localization_safety_monitor
/a2/safe_cmd_vel
    -> gait_controller
    -> Gazebo
```

如果定位状态为 `LOST`、`INITIALIZING` 或里程计异常，安全门会截断速度并
停车。这是导航目标已经发送但机器人不动时首先要检查的环节。

当前 `gait_demo` 是用于仿真和定位验证的解析步态，不是 Unitree 专有的
Sport 控制器，也不是强化学习策略。`locomotion_mode:=dynamic` 只用于接触
动力学实验，不代表已经具备真实 A2 的稳定步态控制能力。

## 七、网页控制台

网页端从安装、启动、建图、保存、定位到导航的完整操作步骤见
[`WEB_RUNBOOK.md`](WEB_RUNBOOK.md)。

### 7.1 启动网页

```bash
bash scripts/setup_web.sh
bash scripts/run_web.sh
```

浏览器访问：

```text
http://127.0.0.1:8080
```

默认账号为 `admin`，默认密码为 `123456`。局域网使用时应通过
`HYY_ADMIN_PASSWORD` 环境变量修改密码，具体见完整网页运行手册。

网页可以完成：

- 启动和停止建图；
- 通过 `/cmd_vel` 手动控制机器人；
- 结束建图并保存 PCD；
- 启动定位和导航；
- 设置 `/initialpose`；
- 发送 `/goal_pose` 导航目标；
- 显示地图、TF、轨迹、机器人位姿和点云；
- 保存和管理网页航点。

网页运行时生成的地图和配置位于 `maps/web/`、`navigation/web/config/`，已加入
`.gitignore`，不会污染源码提交。

如果出现 `address already in use`，表示已有网页服务占用 8080 端口：

```bash
ss -ltnp 'sport = :8080'
pkill -f 'uvicorn robot_server'
```

也可以使用其他端口：

```bash
A2_WEB_PORT=8081 bash scripts/run_web.sh
```

网页和仿真必须使用同一个 ROS 2 环境。系统没有 `python3-venv` 时，安装
脚本会把 Python 3.10 兼容依赖放在 `navigation/web/.python-deps/`，不会使用 conda
Python 3.13 加载 ROS 2 的 `rclpy`。

## 八、常用 ROS 2 接口

| 话题/服务 | 类型 | 用途 |
|---|---|---|
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | 前 JT128 点云；保持为建图和定位输入 |
| `/lidar_points_2` | `sensor_msgs/msg/PointCloud2` | 后 JT128 点云；仅用于导航 |
| `/a2/navigation/lidar_points` | `sensor_msgs/msg/PointCloud2` | 前后雷达各自裁剪至 180° 后的导航点云 |
| `/lidar_imu` | `sensor_msgs/msg/Imu` | FAST-LIO IMU 输入 |
| `/Odometry` | `nav_msgs/msg/Odometry` | FAST-LIO 原始输出 |
| `/a2/odometry` | `nav_msgs/msg/Odometry` | 局部 `odom -> base_link` |
| `/a2/localization` | `nav_msgs/msg/Odometry` | 全局 `map -> base_link` |
| `/a2/map` | `sensor_msgs/msg/PointCloud2` | 已加载的 PCD 地图 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | FAST-LIO 配准点云 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 初始全局位姿 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Nav2 导航目标 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 手动或 Nav2 速度 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 控制器输出 |
| `/a2/safe_cmd_vel` | `geometry_msgs/msg/Twist` | 安全门后的速度 |
| `/a2/relocalization/status` | `std_msgs/msg/String` | 重定位状态 |
| `/a2/odometry/status` | `std_msgs/msg/String` | 里程计健康状态 |
| `/a2/relocalize` | 服务 | 使用最近初值重试重定位 |
| `/a2/relocalization/reset` | 服务 | 清除全局定位并重新初始化 |
| `/map_save` | `std_srvs/srv/Trigger` | 中途保存地图快照 |

查看最终机器人全局位姿：

```bash
ros2 topic echo /a2/localization
ros2 run tf2_ros tf2_echo map base_link
```

## 九、常用诊断命令

```bash
# 传感器频率
ros2 topic hz /lidar_points
ros2 topic hz /lidar_imu

# FAST-LIO 和重定位状态
ros2 topic echo /a2/odometry/status
ros2 topic echo /a2/relocalization/status

# TF 和 Nav2 状态
ros2 run tf2_tools view_frames
ros2 lifecycle get /controller_server
ros2 action list | grep navigate

# 地面/障碍分割频率
ros2 topic hz /ground_segmentation/ground_points
ros2 topic hz /ground_segmentation/obstacle_points

# 检查速度链路
ros2 topic echo /cmd_vel_nav
ros2 topic echo /a2/safe_cmd_vel
```

常见现象和判断：

1. **有目标但不动**：先检查 `/a2/relocalization/status` 是否为
   `LOCALIZED`、`/a2/odometry/status` 是否为 `OK`，再检查
   `/a2/safe_cmd_vel` 是否一直为零。
2. **RViz 报 `frame does not exist`**：检查 `map -> odom -> base_link`
   是否完整，确保定位模式已经启动并且 FAST-LIO 已输出里程计。
3. **机器人走过的位置出现很多雷达点**：这是雷达看到机身、腿或稀疏动态
   点造成的。PCD 投影默认要求每个栅格至少 4 个点，GSeg3D 和 Ground
   Consistency 还会进一步过滤地面和机器人本体点。
4. **导航启动很卡**：降低 JT128 的 `horizontal_samples`，导航默认是
   128；只有 GPU 余量充足时才提高该值。
5. **网页无法启动**：确认只启动了一个 `uvicorn`，并且使用
   `/usr/bin/python3.10` 对应的 ROS 2 Humble 环境。

## 十、仿真与真实机器人边界

仿真中 JT128 和 IMU 的安装位姿由 URDF 和启动参数共同定义，默认安装位置：

```text
lidar_xyz = 0.33767 0.0 0.08134
lidar_rpy = 0.0 0.0 0.0
```

如果实物安装位置不同，需要同时修改 URDF 外参和 FAST-LIO 配置，并确认
Hesai 时间戳、IMU 角速度单位和雷达坐标轴方向。仿真中的
`gait_demo` 只用于算法联调；真实 A2 运行前还需要接入真实底盘控制器、
实测外参和经过验证的步态策略。

定位和导航实现参考了
[FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
的低频全局匹配、高频 FAST-LIO 里程计和全局 TF 融合思路。
