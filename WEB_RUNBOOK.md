# UnitreeA2_Navigation 网页端完整运行手册

本文说明如何从一台干净的 Ubuntu 22.04 + ROS 2 Humble 主机开始，编译工程、
启动网页、完成建图、保存地图、定位并下发导航目标。

> 当前网页的“开始建图”和“启动导航系统”按钮启动的是 **Gazebo 仿真全链路**。
> 真机的两台 JT128 和 A2 底盘不会由这两个按钮自动启动；真机接入要求见
> [真机与双雷达](#八真机与双雷达)。

## 一、网页端启动了什么

网页服务本身只是控制面板。点击按钮后，后端在同一 ROS 2 Domain 中管理以下
进程：

```text
浏览器
  -> Web 后端（8080）
      -> 建图：Gazebo + JT128 仿真 + FAST-LIO
      -> 导航：Gazebo + FAST-LIO + 地图重定位
               + 前后雷达 180° 导航点云 + GSeg3D + Nav2 + 安全门
```

网页与 ROS 的主要接口：

| 网页操作 | ROS 2 接口 |
|---|---|
| 手动移动 | `/cmd_vel` |
| 设置初始位姿 | `/initialpose`，坐标系 `map` |
| 下发导航目标 | `/goal_pose`，坐标系 `map` |
| 前雷达点云 | `/lidar_points` |
| 后雷达点云 | `/lidar_points_2` |
| 导航使用的双雷达点云 | `/a2/navigation/lidar_points` |
| 定位结果 | `/a2/localization` 和 `map -> base_link` TF |

建图、存图和导航三种状态互斥。切换地图前也必须先停止导航和建图。

## 二、首次安装和编译

以下步骤只需在首次部署、拉取了新依赖或修改 ROS 包后执行。

```bash
cd /home/gao/Documents/UnitreeA2_Navigation
source /opt/ros/humble/setup.bash

./scripts/install_dependencies.sh
./scripts/fetch_navigation_dependencies.sh
./scripts/build_ros2.sh
bash scripts/setup_web.sh
```

说明：

- `install_dependencies.sh` 会使用 `sudo apt` 安装 ROS、Nav2、PCL 等依赖；
- `fetch_navigation_dependencies.sh` 会下载固定版本的 GSeg3D 和 Nav2 地形插件；
- `build_ros2.sh` 编译 `localization/`、`navigation/`、`driver/` 三个模块；
- `setup_web.sh` 创建网页 Python 环境并安装依赖。

编译成功后可确认关键包存在：

```bash
source install/setup.bash
ros2 pkg prefix a2_localization_bringup
ros2 pkg prefix a2_dual_lidar_nav
ros2 pkg prefix a2_terrain_nav
ros2 pkg prefix a2_hesai_driver
```

## 三、每次启动网页

在工程根目录执行：

```bash
cd /home/gao/Documents/UnitreeA2_Navigation
bash scripts/run_web.sh
```

这个终端需要一直保持运行。脚本会自动加载 ROS 2 Humble、当前工作空间的
`install/setup.bash`，并启动网页后端。

浏览器访问：

- 本机：`http://127.0.0.1:8080`
- 局域网其他电脑：`http://<运行网页主机的 IP>:8080`

默认登录信息：

```text
用户名：admin
密码：123456
```

在局域网公开端口前，建议通过环境变量修改登录密码：

```bash
HYY_ADMIN_PASSWORD='替换为强密码' \
HYY_ADVANCED_PASSWORD='替换为另一强密码' \
bash scripts/run_web.sh
```

需要更换端口时：

```bash
A2_WEB_PORT=8081 bash scripts/run_web.sh
```

然后访问 `http://127.0.0.1:8081`。如果其他电脑无法打开，先确认主机 IP、
防火墙和端口监听：

```bash
hostname -I
ss -ltnp 'sport = :8080'
```

## 四、最快的仿真导航流程

工程已有可用地图时，按下面顺序操作：

1. 启动网页并登录。
2. 打开“地图管理”，在地图列表中选择要使用的地图。
3. 返回“控制面板”，点击“启动导航系统”。
4. 等待页面显示 ROS 已连接、地图已加载、TF 正常、导航已就绪。
5. 默认 `auto_initialize: true`，可重复仿真会自动尝试初始化。若尚未定位，
   点击“初始位姿”，在三维地图中按下并拖动，给出大致位置和朝向。
6. 点击“导航目标”，在地图中按下并拖动设置目标位置和朝向。
7. 观察规划路径、机器人轨迹、局部/全局代价地图以及导航状态。
8. 结束后先点击“停止运动”，再点击“停止导航系统”。

“启动导航系统”会执行：

```bash
ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py \
  map_pcd:=<当前地图 PCD> \
  map_yaml:=<当前地图 YAML> \
  auto_initialize:=true \
  locomotion_mode:=gait_demo \
  rmw_implementation:=rmw_fastrtps_cpp \
  gui:=false rviz:=false
```

因此不需要再在另一个终端手动启动 Gazebo、FAST-LIO、GSeg3D 或 Nav2。
网页默认关闭 Gazebo GUI 和 RViz，以降低资源占用；点云、地图、机器人姿态和
路径直接显示在网页中。

## 五、网页建图和保存地图

### 5.1 开始建图

1. 确认导航系统已经完全停止。
2. 打开“地图管理”。
3. 在“建图与存图”中输入新地图名，例如 `floor2`。
4. 点击“开始建图”。
5. 返回控制面板，通过方向控制区移动机器人，松开按钮会发送零速度。
6. 让机器人平稳覆盖需要导航的区域，避免长时间原地快速旋转。

网页后端实际启动：

```bash
ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py \
  operation_mode:=mapping \
  map_path:=<新地图目录>/a2_map.pcd \
  trajectory:=manual \
  locomotion_mode:=gait_demo \
  gui:=false rviz:=false
```

### 5.2 完成并保存

采集完成后：

1. 松开运动按钮并点击“停止运动”；
2. 回到“地图管理”；
3. 点击“完成并保存地图”；
4. 等待页面提示“二维地图与 PCD 已保存，建图已停止”；
5. 在地图列表中检查新地图文件是否完整。

保存流程会调用 `/map_save`，并自动将 PCD 投影为 Nav2 使用的 YAML/PGM。
每张网页地图保存在：

```text
maps/web/<地图名>/
├── a2_map.pcd
├── a2_nav2_map.yaml
└── a2_nav2_map.pgm
```

“放弃并停止”只停止当前建图，不执行最终地图转换。

## 六、定位、导航和航点

### 6.1 切换地图

先停止导航和建图，再到“地图管理”选择地图。若页面提示需要重启导航，返回
控制面板点击“启动导航系统”即可。

### 6.2 设置初始位姿

导航需要完整的：

```text
map -> odom -> base_link
```

如果自动初始化没有成功：

1. 让机器人静止；
2. 点击“初始位姿”；
3. 在地图中机器人实际位置按下鼠标并拖向机头方向；
4. 等待定位状态变为 `LOCALIZED`、TF 变为正常。

位置误差过大、朝向相反或 PCD 与当前环境不一致都会导致重定位被拒绝。此时
重新设置更准确的初始位姿，不要连续下发导航目标。

### 6.3 下发单点目标

定位完成后点击“导航目标”，在地图中按下并拖动设置终点和朝向。自动导航
期间网页手动遥控会被锁定，以避免 `/cmd_vel` 命令冲突。

### 6.4 航点与导览

在“存点管理”中可以记录当前 `map` 坐标位姿、编辑名称、调整顺序并执行往返
测试。航点按地图分区保存在：

```text
navigation/web/config/navigation_data.json
```

只有地图已经加载且 `map -> base_link` 有效时才能记录当前位姿。

## 七、前后雷达在导航中的处理

导航阶段的点云路径是：

```text
/lidar_points   （前雷达）-- 各自雷达坐标系前向 180° --┐
                                                       ├-> /a2/navigation/lidar_points
/lidar_points_2 （后雷达）-- 各自雷达坐标系前向 180° --┘
                                                               -> GSeg3D -> Nav2
```

- 两路点云分别按其自身 `frame_id` 的前向半平面裁剪，再变换到 `base_link`；
- 两路输入独立处理，任意一路暂时无数据时，另一路仍可用于导航；
- 前雷达原始 `/lidar_points` 仍单独供 FAST-LIO 建图和定位使用；
- 后雷达 `/lidar_points_2` **只进入导航感知**，不修改建图和定位；
- 当前 Gazebo 启动文件只模拟前雷达，因此仿真中后雷达话题为空，导航仍可依靠
  前雷达运行；双雷达覆盖需要真机或额外的后雷达发布器。

网页“点云组件”可添加或选择以下话题进行检查：

```text
/lidar_points
/lidar_points_2
/a2/navigation/lidar_points
/ground_segmentation/ground_points
/ground_segmentation/obstacle_points
/cloud_registered
```

## 八、真机与双雷达

当前网页的一键命令是仿真配置。真机试运行前必须先完成以下工作：

1. 为前、后两台 JT128 分别配置不同的设备 IP、UDP 端口和主机网卡地址；
2. 前雷达发布 `/lidar_points` 和 `/lidar_imu`；
3. 后雷达发布 `/lidar_points_2`；
4. 两路点云的 `header.frame_id` 必须准确；
5. 发布两个雷达坐标系到 `base_link` 的静态 TF；
6. 接入并验证真实 A2 底盘对安全速度命令的执行；
7. 将网页后端的启动命令从当前 Gazebo launch 切换为真机 bringup 后再使用
   “启动导航系统”。

工程提供的 `driver/a2_hesai_driver/config/jt128.yaml` 是**单台前雷达模板**，
不能原样同时启动两台雷达。双雷达真机配置需要分别设置话题、frame、IP 和端口，
并确保节点名不冲突。

在真机启动网页控制前，至少确认：

```bash
source /opt/ros/humble/setup.bash
source /home/gao/Documents/UnitreeA2_Navigation/install/setup.bash

ros2 topic hz /lidar_points
ros2 topic hz /lidar_points_2
ros2 topic hz /lidar_imu
ros2 run tf2_ros tf2_echo base_link <前雷达frame>
ros2 run tf2_ros tf2_echo base_link <后雷达frame>
```

还应把 `navigation/web/config/unitree_a2.yaml` 中的
`web_runtime.auto_initialize` 设为 `false`，由操作员在网页上明确设置初始位姿。
真实机器人周围必须留有急停人员和安全空间；当前 `gait_demo` 是仿真控制器，
不能直接替代 Unitree 真机运动控制接口。

## 九、运行状态检查

在另一个终端加载相同 ROS 环境：

```bash
cd /home/gao/Documents/UnitreeA2_Navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

然后检查：

```bash
# 雷达与 IMU
ros2 topic hz /lidar_points
ros2 topic hz /lidar_points_2
ros2 topic hz /lidar_imu

# 双雷达导航输出和地形分割
ros2 topic hz /a2/navigation/lidar_points
ros2 topic hz /ground_segmentation/ground_points
ros2 topic hz /ground_segmentation/obstacle_points

# 定位状态和 TF
ros2 topic echo /a2/odometry/status
ros2 topic echo /a2/relocalization/status
ros2 run tf2_ros tf2_echo map base_link

# Nav2 与速度安全链
ros2 lifecycle get /controller_server
ros2 action list | grep navigate
ros2 topic echo /cmd_vel_nav
ros2 topic echo /a2/safe_cmd_vel
```

网页和这些诊断终端必须使用相同的 `ROS_DOMAIN_ID` 与 RMW 实现。默认是：

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 十、停止顺序

正常停止：

1. 点击“停止运动”；
2. 取消正在执行的导航或导览任务；
3. 点击“停止导航系统”；
4. 等待页面确认所有导航进程已经退出；
5. 最后在运行 `run_web.sh` 的终端按 `Ctrl+C` 停止网页。

建图时不要直接关闭网页或强制杀死进程。需要地图时使用“完成并保存地图”，
不需要保存时使用“放弃并停止”。

## 十一、常见问题

### 网页打不开

```bash
ss -ltnp 'sport = :8080'
```

若端口已被旧网页占用，先回到旧进程终端按 `Ctrl+C`。确认没有需要保留的任务
后，才可结束残留网页进程。

### 页面打开但 ROS 未连接

确认工程已经编译，并且 `scripts/web_ros_env.sh` 能找到：

```text
/opt/ros/humble/setup.bash
/home/gao/Documents/UnitreeA2_Navigation/install/setup.bash
```

同时检查网页终端是否有 Python 或 `rclpy` 导入错误。

### 已发送目标但机器人不动

依次检查：

1. `/a2/relocalization/status` 是否为 `LOCALIZED`；
2. `/a2/odometry/status` 是否为 `OK`；
3. `map -> odom -> base_link` 是否完整；
4. `/cmd_vel_nav` 是否有非零速度；
5. `/a2/safe_cmd_vel` 是否被安全门置零。

### 后雷达没有参与导航

检查 `/lidar_points_2` 是否有频率、消息 `frame_id` 是否正确、该 frame 到
`base_link` 的 TF 是否存在。仿真默认没有后雷达发布器，这是预期行为。

### 地图保存失败

确认 `/map_save` 服务存在，PCD 输出目录可写，且建图期间 `/cloud_registered`
持续更新。地图保存最长会等待约 35 秒，不要在保存过程中停止网页后端。

### 页面卡顿

减少同时显示的点云图层，关闭不需要的原始点云，只保留地图、机器人、路径和
必要的地面/障碍点云。浏览器渲染高密度 JT128 点云时会显著占用 GPU。

## 十二、相关文件

- 工程总说明：`README.md`
- 定位模块：`localization/README.md`
- 导航模块：`navigation/README.md`
- 驱动模块：`driver/README.md`
- 网页配置：`navigation/web/config/unitree_a2.yaml`
- 网页启动命令：`navigation/web/robot_server/app/services/process_manager.py`
- 网页地图目录：`maps/web/`
