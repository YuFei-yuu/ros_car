# ROS2 小车课程设计最低目标包

`course_design` 是课程设计的薄封装包，不直接改厂家 `slam`、`navigation`、`example` 等源码。它把最低目标拆成三个可单独启动的链路：建图、导航到点、决策树巡逻。

## 实现方式

- 建图：复用 `slam_toolbox` 的二维激光 SLAM，订阅 `/scan` 构建 `/map`，通过 `/slam_toolbox/save_map` 保存地图。
- 定位：复用 Nav2 中的 AMCL 粒子滤波定位。
- 导航：复用 Nav2，包含 Navfn 全局规划和 TEB/DWB 局部控制器。
- 避障：复用 Nav2 global/local costmap 的 static layer、obstacle/voxel layer 和 inflation layer。静态障碍来自已保存地图，动态障碍来自导航时的激光雷达 `/scan`。
- 二维码：复用 OpenCV `QRCodeDetector`，订阅 `/depth_cam/rgb0/image_raw`，识别地面二维码内容并发布目标点名。
- 决策树：`behavior_node` 用简单状态机执行 `qrcode_target`、`go_named_point`、`return_home`、`patrol`。启动后先等待 RViz 的 `2D Pose Estimate` 初始位姿和人工确认，再运行预设任务。
- 可视化：三个 launch 默认启动 RViz。导航和决策树会发布 `/waypoints`，在 RViz 中显示配置文件里的命名点和当前目标点。

## 配置文件

配置文件位置：

```bash
/home/ubuntu/ros2_ws/src/course_design/config/course_design.yaml
```

现场主要修改：

- `map_name`: 默认 `map_01`
- `use_teb`: 默认 `true`
- `waypoints`: 修改 `home`、`pick_area`、`goal_red`、`goal_green`、`goal_blue` 的 `x/y/yaw_deg`
- `navigation.timeout_sec`: 单次导航超时
- `behavior.patrol_points`: 巡逻的两个点
- `behavior.patrol_count`: 往返巡逻次数
- `behavior.require_initial_pose`: 默认 `true`，要求先在 RViz 中设置初始位姿
- `behavior.start_topic`: 默认 `/behavior_start`，发布 `std_msgs/msg/Bool` 的 `true` 后启动默认任务
- `behavior.reset_topic`: 默认 `/behavior_reset`，小车到达二维码目标后发布 `true` 进入下一轮
- `qrcode.image_topic`: 默认 `/depth_cam/rgb0/image_raw`
- `qrcode.target_topic`: 默认 `/qrcode/target`
- `qrcode.timeout_sec`: 默认 `0.0`，表示一直等待二维码目标
- `qrcode.allowed_targets`: 二维码允许输出的目标点名

地图保存固定为：

```bash
/home/ubuntu/ros2_ws/src/slam/maps/map_01
```

## 启动命令

构建：

```bash
cd /home/ubuntu/ros2_ws
colcon build --packages-select course_design --parallel-workers 1
source install/setup.bash
```

停用出厂服务
```bash
sudo systemctl stop start_app_node.service
```

建图：

```bash
ros2 launch course_design mapping.launch.py
```

该命令会默认打开 RViz，使用 `slam/rviz/slam_desktop.rviz` 观察 `/map`、`/scan` 和 TF。

保存地图：

```bash
ros2 service call /map_status_node/save_map std_srvs/srv/Trigger "{}"
```

导航到点：

```bash
ros2 launch course_design course_nav.launch.py
```

该命令会默认打开 RViz，使用 `navigation/rviz/navigation_desktop.rviz` 观察地图、AMCL、global/local costmap、规划路径和 `/waypoints`。可以使用 RViz 顶部工具栏的 Nav2 Goal 工具直接发布目标点导航。

示例服务调用：

```bash
ros2 service call /waypoint_nav_node/go_home std_srvs/srv/Trigger "{}"
ros2 service call /waypoint_nav_node/go_pick_area std_srvs/srv/Trigger "{}"
ros2 service call /waypoint_nav_node/go_named_point interfaces/srv/SetString "{data: goal_green}"
```

决策树巡逻：

```bash
ros2 launch course_design behavior.launch.py
```

`behavior.launch.py` 默认会打开 RViz，并启动深度相机、二维码识别节点和导航行为树，但不会立刻移动。先在 RViz 顶部工具栏点击 `2D Pose Estimate`，在地图上拖拽设置机器人初始位置和朝向。确认位姿后，在新终端发送启动指令：

```bash
ros2 service call /behavior_node/start std_srvs/srv/Trigger "{}"
```

服务会立即返回，行为树在后台持续运行。也可以通过话题启动：

```bash
ros2 topic pub --once /behavior_start std_msgs/msg/Bool "{data: true}"
```

默认任务当前为 `qrcode_target`，任务流为：

1. 启动后先导航到 `home`。
2. 到达 `home` 后等待二维码识别，二维码内容必须是 `pick_area`、`goal_red`、`goal_green` 或 `goal_blue`。
3. 识别到目标点后，导航前往二维码指定目标。
4. 到达目标后进入等待重置状态。
5. 收到重置指令后回到 `home`。
6. 回到 `home` 后重新识别二维码、导航到目标、等待重置，循环执行。

到达二维码目标后，在新终端发送重置指令：

```bash
ros2 service call /behavior_node/reset std_srvs/srv/Trigger "{}"
```

也可以通过话题重置：

```bash
ros2 topic pub --once /behavior_reset std_msgs/msg/Bool "{data: true}"
```

可观察命名点、当前目标点、全局路径、局部路径、costmap 和相机图像。手动触发指定任务：

```bash
ros2 service call /behavior_node/run_task interfaces/srv/SetString "{data: qrcode_target}"
ros2 service call /behavior_node/run_task interfaces/srv/SetString "{data: patrol}"
ros2 service call /behavior_node/run_task interfaces/srv/SetString "{data: return_home}"
ros2 service call /behavior_node/run_task interfaces/srv/SetString "{data: go_named_point:pick_area}"
```

二维码识别结果调试：

```bash
ros2 topic echo /qrcode/target
ros2 topic echo /qrcode/image_result
```

## 调试顺序

1. 启动 `mapping.launch.py`，确认终端持续输出 `/map` 和 `/scan` 状态。
2. 调用 `/map_status_node/save_map`，确认生成 `map_01.yaml` 和 `map_01.pgm`。
3. 修改 `course_design.yaml` 中的目标点坐标。
4. 启动 `course_nav.launch.py`，分别测试 `go_home` 和 `go_pick_area`。
5. 启动 `behavior.launch.py`，在 RViz 中用 `2D Pose Estimate` 设置初始位姿。
6. 调用 `/behavior_node/start` 或发布 `/behavior_start`，启动默认二维码目标任务。
7. 到达二维码目标后，调用 `/behavior_node/reset` 或发布 `/behavior_reset`，验证循环进入下一轮。

调试阶段必须有人看守。任一导航失败、超时或流程结束时，节点会发布零速度到配置中的停车话题。
