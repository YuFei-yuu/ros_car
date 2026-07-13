# YOLO 巡检任务现场操作说明

本文档面向现场调试人员，说明如何启动、测试和处理 `yolo_patrol` 任务中的常见问题。核心原则是：先验证相机和 YOLO，再验证导航，最后运行完整巡检闭环。

## 1. 基础环境 source

每个新终端建议先执行：

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

如果当前终端是 bash，则使用：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 单独测试相机和 YOLO

第一步先确认 RGB 相机和 YOLO 可以正常工作。

在 NoMachine 桌面终端运行：

```bash
ros2 launch example yolo_detect.launch.py model_name:=yolo26n conf:=0.25 display:=true
```

另开终端启动 YOLO：

```bash
ros2 service call /yolo/start std_srvs/srv/Trigger "{}"
```

查看 RGB 图像频率：

```bash
ros2 topic hz /depth_cam/rgb0/image_raw
```

查看 YOLO 检测结果：

```bash
ros2 topic echo /yolo/object_detect
```

建议先测试：

| 目标 | 预期类别 | 说明 |
|---|---|---|
| 人 | person | 通常识别最稳定 |
| 竖立矿泉水瓶 | bottle | 横躺透明瓶不稳定，不作为第一版强验收目标 |
| 手机/遥控器 | cell phone / remote | 类别可能跳变，可合并为疑似电子设备 |

停止 YOLO：

```bash
ros2 service call /yolo/stop std_srvs/srv/Trigger "{}"
```

如果只想停止窗口和 launch，直接在启动 YOLO 的终端按：

```text
Ctrl + C
```

## 3. 单独测试导航

启动导航：

```bash
ros2 launch course_design course_nav.launch.py
```

```text
2D Pose Estimate
```

设置小车初始位姿。

如果已经配置了专用巡检点，可以继续测试：

```bash
ros2 service call /waypoint_nav_node/go_named_point interfaces/srv/SetString "{data: inspect_1}"
ros2 service call /waypoint_nav_node/go_named_point interfaces/srv/SetString "{data: inspect_2}"
ros2 service call /waypoint_nav_node/go_named_point interfaces/srv/SetString "{data: inspect_3}"
```

如果某个点无法到达，先修改点位，不要直接运行完整巡检。

## 6. 记录巡检点位置和朝向

如果需要重新配置 `inspect_1 / inspect_2 / inspect_3`，推荐流程：

1. 启动 `course_nav.launch.py`。
2. 在 RViz 中设置初始位姿。
3. 通过 RViz 或命名点导航让小车移动到合适位置。
4. 读取当前位置与朝向。

优先使用 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

如果报 frame 不存在，尝试：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

记录输出中的：

```text
Translation: [x, y, z]
Rotation in RPY (degree): [..., ..., yaw]
```

然后写入：

```yaml
inspect_1:
  x: 读取到的x
  y: 读取到的y
  yaw_deg: 读取到的yaw角
```

如果 `/amcl_pose` 可用，也可以读取：

```bash
ros2 topic echo /amcl_pose --once
```

但实际使用中 TF 通常更直接。

## 7. 启动完整 YOLO 巡检任务

完整启动：

```bash
ros2 launch yolo_patrol yolo_patrol.launch.py
```

如果你已经单独启动了导航和 YOLO，可以避免重复启动：

```bash
ros2 launch yolo_patrol yolo_patrol.launch.py start_navigation:=false start_yolo:=false
```

启动后，在 RViz 中设置初始位姿，然后发送开始指令：

```bash
ros2 topic pub --once /yolo_patrol/start std_msgs/msg/Bool "{data: true}"
```

监听状态：

```bash
ros2 topic echo /yolo_patrol/status
```

## 8. 不同情况如何处理

### 8.1 正常继续巡逻

如果状态显示没有关注目标，或识别到 `bottle` 并记录继续，人员无需发送额外指令。

典型状态：

```text
no_target
log_continue
DONE
```

操作：

```text
观察即可，不需要发 topic。
```

### 8.2 识别到 person，安全停车

含义：

```text
巡检区域有人，系统进入 safety_stop / WAIT_RESET。
```

人员操作：

1. 确认小车周围安全；
2. 人员离开摄像头画面；
3. 发送 reset 继续巡逻。

```bash
ros2 topic pub --once /yolo_patrol/reset std_msgs/msg/Bool "{data: true}"
```

也可以调用服务：

```bash
ros2 service call /yolo_patrol_node/reset std_srvs/srv/Trigger "{}"
```

### 8.3 识别到 cell phone / remote，告警停车

含义：

```text
疑似电子设备或遗留物，系统进入 alert_stop / WAIT_RESET。
```

人员操作：

1. 记录检测类别和置信度；
2. 确认是否为测试物；
3. 如确认无风险，发送 reset 继续。

```bash
ros2 topic pub --once /yolo_patrol/reset std_msgs/msg/Bool "{data: true}"
```

如果需要终止测试：

```text
在 launch 终端按 Ctrl + C
```

然后补发零速度：

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

### 8.4 到点后没有识别到目标

可能状态：

```text
no_target
```

含义：

```text
YOLO 有消息，但没有关注类别满足阈值。
```

人员操作：

- 如果现场确实没有目标：无需处理；
- 如果现场有目标但没识别到：调整目标姿态、光照、距离，或降低阈值。

可修改配置：

```yaml
detection:
  observe_time_sec: 8.0
  min_score: 0.50
  stable_min_count: 3
```

如果当前任务停在等待状态，发送：

```bash
ros2 topic pub --once /yolo_patrol/reset std_msgs/msg/Bool "{data: true}"
```

### 8.5 移动中看到目标，到点后没确认

可能状态：

```text
missed_after_moving
```

含义：

```text
移动中轻量监听曾看到候选目标，但停车检测没有多帧确认。
```

人员操作：

1. 检查巡检点朝向 `yaw_deg` 是否合适；
2. 检查目标是否在摄像头视野中央；
3. 适当增加观察时间；
4. 必要时重新记录该巡检点位姿。

常用修改：

```yaml
detection:
  observe_time_sec: 8.0
  retry_observe_time_sec: 4.0
```

### 8.6 没有 YOLO 数据

可能状态：

```text
no_yolo_data
```

含义：

```text
停车检测期间没有收到 /yolo/object_detect 消息。
```

人员操作：

检查话题：

```bash
ros2 topic list | grep yolo
ros2 topic echo /yolo/object_detect
```

启动 YOLO：

```bash
ros2 service call /yolo/start std_srvs/srv/Trigger "{}"
```

检查相机：

```bash
ros2 topic hz /depth_cam/rgb0/image_raw
```

如果相机或 YOLO 卡住，停止当前 launch 后重新启动：

```text
Ctrl + C
```

再运行：

```bash
ros2 launch example yolo_detect.launch.py model_name:=yolo26n conf:=0.25 display:=true
```

### 8.7 导航失败

可能原因：

- 初始位姿不准；
- 巡检点坐标在障碍物内；
- 地图与现场不一致；
- Nav2 costmap 认为路径不可达；
- 小车电量低。

人员操作：

1. 停止当前任务；
2. 发布零速度；
3. 单独测试目标点。

```text
Ctrl + C
```

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

重新测试：

```bash
ros2 launch course_design course_nav.launch.py
ros2 service call /waypoint_nav_node/go_named_point interfaces/srv/SetString "{data: inspect_1}"
```


如果还能发命令，先停车：

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 9. 推荐测试表

| 点位 | 放置目标 | 预期识别 | 预期动作 | 人员操作 |
|---|---|---|---|---|
| inspect_1 | 人进入画面 | person | 安全停车 | 确认安全后发送 `/yolo_patrol/reset` |
| inspect_2 | 竖立矿泉水瓶 | bottle | 记录继续 | 无需操作 |
| inspect_3 | 手机或遥控器 | cell phone / remote | 告警停车 | 记录结果后发送 `/yolo_patrol/reset` |

## 10. 常用命令汇总

启动 YOLO 检测：

```bash
ros2 service call /yolo/start std_srvs/srv/Trigger "{}"
```

停止 YOLO 检测：

```bash
ros2 service call /yolo/stop std_srvs/srv/Trigger "{}"
```

启动 YOLO 巡检：

```bash
ros2 topic pub --once /yolo_patrol/start std_msgs/msg/Bool "{data: true}"
```

人工确认继续：

```bash
ros2 topic pub --once /yolo_patrol/reset std_msgs/msg/Bool "{data: true}"
```

监听巡检状态：

```bash
ros2 topic echo /yolo_patrol/status
```

监听 YOLO 输出：

```bash
ros2 topic echo /yolo/object_detect
```

强制发布零速度：

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

读取小车当前地图位姿：

```bash
ros2 run tf2_ros tf2_echo map base_link
```
