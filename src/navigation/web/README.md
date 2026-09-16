# Unitree A2 Web Console

本目录是从 `robot_navigation-main` 移植的网页控制台，已经将机器人配置、ROS 2 话题和启动命令替换为当前 Unitree A2 工程：Hesai JT128、FAST-LIO、GSeg3D 和 Nav2。

从依赖安装、工程编译、网页建图、地图保存到定位导航的完整中文运行手册见
[`../../../WEB_RUNBOOK.md`](../../../WEB_RUNBOOK.md)。

## 启动

在工程根目录执行：

```bash
bash scripts/setup_web.sh
bash scripts/run_web.sh
```

浏览器打开 `http://127.0.0.1:8080`，默认账号为 `admin`、默认密码为
`123456`。局域网使用时应通过 `HYY_ADMIN_PASSWORD` 环境变量修改密码。
网页服务会自动 source `/opt/ros/humble` 和当前工程的 `install/setup.bash`。

如果系统没有安装 `python3-venv`，安装脚本会把 Python 3.10 兼容 wheel 放到被
`.gitignore` 忽略的 `src/navigation/web/.python-deps/`，不会使用 conda 的
Python 3.13 去加载 ROS 2 的 `rclpy`。

## 网页功能与 ROS 2 对接

- 启动/停止定位导航：`a2_terrain_nav/a2_terrain_navigation.launch.py`。
- 启动建图：`a2_localization_bringup/a2_jt128_gazebo.launch.py operation_mode:=mapping`。
- 网页速度控制发布 `/cmd_vel`，导航目标发布到 Nav2 当前使用的 `/goal_pose`
  （`geometry_msgs/msg/PoseStamped`，坐标系为 `map`）。
- 网页航点队列由后端直接按顺序调用 Nav2，不依赖原项目的 HYY waypoint/tcp 节点。
- 默认显示 `/cloud_registered`，可切换 `/lidar_points`、GSeg3D 地面/障碍点云和 `/Laser_map`。
- TF、机器人位姿统一使用 `map/odom -> base_link`，不再使用原项目的 `base_link_2d`。
- 地图文件保存在工程内 `maps/web/<地图名>/`；该运行时目录已加入 `.gitignore`。

## 地图流程

网页“建图”启动后使用手动 `/cmd_vel` 驱动机器人；点击“结束并保存”会调用
`/map_save`，随后将 PCD 转换为 Nav2 的 YAML/PGM。网页地图选择和初始位姿/
航点数据保存在 `src/navigation/web/config/navigation_data.json`。

可选环境变量：

```bash
A2_WEB_HOST=0.0.0.0 A2_WEB_PORT=8080 bash scripts/run_web.sh
```
