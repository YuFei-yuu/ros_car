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
- `transport.target_color`: 兼容旧配置的单色搬运目标，默认 `red`
- `transport.color_sequence`: 搬运颜色顺序，当前示例为 `[red, green, blue]`
- `transport.goal_by_color`: 目标颜色到导航点的映射，例如 `red: goal_red`
- `transport.backoff_speed_mps`: 抓取后的后退速度，当前示例为 `0.06 m/s`
- `transport.backoff_duration_sec`: 抓取后的后退持续时间，当前示例为 `2.0 s`
- `transport.backoff_period_sec`: 后退速度指令发布周期，当前示例为 `0.05 s`
- `vision.target_pixel`、`vision.*_tolerance_px`: 取货区视觉对准目标像素和容差
- `vision.angular_gain`、`vision.linear_gain`: 视觉控制增益；现场标定时可调整正负方向
- `vision.max_*_speed`: 视觉对准的低速限幅
- `arm.*_action`、`arm.*_timeout_sec`: 机械臂动作组及其超时保护

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

## 多颜色自主搬运

搬运流程使用 `transport.color_sequence` 指定颜色顺序，并通过
`transport.goal_by_color` 找到每种颜色对应的目标点。默认流程为：

1. 在 RViz 设置初始位姿。
2. 机械臂执行 `navigation_pick_init`。
3. 按 `color_sequence` 顺序导航到 `pick_area`，识别并对准目标颜色，执行 `navigation_pick`。
4. 抓取后按 `backoff_speed_mps` 后退 `backoff_duration_sec`。
5. 导航到该颜色在 `goal_by_color` 中配置的目标点，执行 `navigation_place`。
6. 重复步骤 3 至 5，直到列表中的颜色全部完成。
7. 返回 `home`，机械臂回到安全姿态并进入 `DONE`。

完整流程使用已保存地图，不启动 SLAM。启动前确认 `course_design.yaml` 中的 `home`、`pick_area`、目标点坐标、颜色顺序、后退参数以及视觉对准参数已经现场标定。

当前示例后退参数为 `0.06 m/s` 持续 `2.0 s`

1. 单独调试颜色识别、对准和机械臂取放：

```bash
ros2 launch course_design color_pick.launch.py
```

该命令会同时启动 RViz，并自动加载 `color_pick.rviz`，在 Image 面板中显示
`/color_detect/image_result`。该 launch 仍然只启动相机、底盘、舵机和颜色检测，
不启动 Nav2 或 SLAM。

2. 标定 LAB 颜色阈值。保持物块和背景处于实际工作光照下运行：

```bash
bash /home/ubuntu/software/lab_tool/lab_tool.sh
```

3. 检查相机和检测结果话题。RViz 会自动显示 `/color_detect/image_result`，也可以用 `rqt_image_view` 查看原图`/depth_cam/rgb0/image_raw`或结果图`/color_detect/image_result`。

```bash
ros2 run rqt_image_view rqt_image_view
```

4. 配置并验证检测颜色。服务返回 `success: True` 后，结果图才会绘制该颜色的检测框。

```bash
ros2 service call /pick_place_node/set_target_color interfaces/srv/SetString "{data: red}"
```

5. 标定视觉对准位置。观察 `color_info` 中物块中心的 `x`、`y`，将机械臂实际可抓取时的中心位置写入 `course_design.yaml`：

```yaml
vision:
  target_pixel:
    x: 320
    y: 388
```

同时调整 `x_tolerance_px`、`y_tolerance_px`、`angular_gain`、`linear_gain` 和 `max_*_speed`。方向错误时反转对应增益符号；每次只调整一个参数，修改后重启节点。

6. 在另一终端初始化机械臂并执行抓取：

```bash
ros2 service call /pick_place_node/prepare std_srvs/srv/Trigger "{}"
ros2 service call /pick_place_node/pick std_srvs/srv/Trigger "{}"
ros2 service call /pick_place_node/place std_srvs/srv/Trigger "{}"
```

7. 任意阶段停止：

```bash
ros2 service call /pick_place_node/stop std_srvs/srv/Trigger "{}"
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{}"
```

`set_target_color` 会立即配置 `/color_detect/set_param`。服务返回成功后，
`/color_detect/image_result` 才会开始绘制目标颜色的检测框；如果返回检测器不可用，先检查
`ros2 service list | grep color_detect`，不要直接调用 `pick`。


8. 完整搬运流程：

```bash
ros2 launch course_design transport.launch.py
```

该命令会打开 RViz，但不会立刻移动。使用 RViz 顶部的 `2D Pose Estimate` 设置初始位姿后，启动并观察状态：

```bash
ros2 service call /transport_workflow_node/start std_srvs/srv/Trigger "{}"
ros2 topic echo /transport_workflow/state
```

也可以用话题启动，或在任何阶段取消流程：

```bash
ros2 topic pub --once /transport_start std_msgs/msg/Bool "{data: true}"
ros2 service call /transport_workflow_node/cancel std_srvs/srv/Trigger "{}"
ros2 topic pub --once /transport_cancel std_msgs/msg/Bool "{data: true}"
```

取消、检测超时、动作组缺失、动作超时或导航失败时，工作流会取消导航、发布零速度并尝试让机械臂回到安全姿态。完整流程运行期间必须有人看守。

## 调试顺序

1. 启动 `mapping.launch.py`，确认终端持续输出 `/map` 和 `/scan` 状态。
2. 调用 `/map_status_node/save_map`，确认生成 `map_01.yaml` 和 `map_01.pgm`。
3. 修改 `course_design.yaml` 中的目标点坐标。
4. 启动 `course_nav.launch.py`，分别测试 `go_home` 和 `go_pick_area`。
5. 启动 `behavior.launch.py`，在 RViz 中用 `2D Pose Estimate` 设置初始位姿。
6. 调用 `/behavior_node/start` 或发布 `/behavior_start`，启动默认二维码目标任务。
7. 到达二维码目标后，调用 `/behavior_node/reset` 或发布 `/behavior_reset`，验证循环进入下一轮。
8. 启动 `color_pick.launch.py`，完成目标像素、容差、控制增益和速度上限的标定。
9. 启动 `transport.launch.py`，依次验证每种颜色的 `GO_PICK_AREA`、`PICK`、`BACKOFF`、`GO_GOAL`、`PLACE`，以及最后的 `RETURN_HOME`、`SAFE`、`DONE` 状态。
