# ROS2 小车课程设计实施规范

本文约束 `plan.md` 的具体实现方式。`plan.md` 讲做什么，本文讲在当前环境里怎样做、哪些边界不能踩。

## 1. 当前环境

已确认环境：

- Ubuntu 22.04.5 LTS，`jammy`
- ROS 2 Humble，目录 `/opt/ros/humble`
- Python 3.10.12
- 平台 `aarch64`，Tegra 内核
- RMW：`rmw_fastrtps_cpp`
- `MACHINE_TYPE=ROSOrin_Mecanum_Pro`
- `DEPTH_CAMERA_TYPE=aurora`
- `LIDAR_TYPE=SCLIDAR`
- `need_compile=False`
- OpenCV 4.11.0，NumPy 1.26.4
- `rclpy`、`cv_bridge`、`nav2_simple_commander` 可用

注意：

- 不用 `ros2 --version` 判断版本，当前 CLI 不支持该参数。
- `colcon version-check` 会尝试联网，当前环境会失败，不作为验收项。
- OpenCV 在无显示/GPU 上下文中可能打印 NvRM/EGL 警告，导入成功即可。

## 2. 代码边界

- 新增代码使用 ROS2 Python `rclpy` 和 `ament_python`。
- 不使用 ROS1：禁止 `rospy`、`catkin`、`ServiceProxy`。
- 不新增 C++ 节点。
- 不新增自定义 msg/srv，优先复用 `interfaces`。
- 不从网络下载依赖、模型或系统包。
- 不直接改厂家 `example`、`slam`、`navigation`、`app` 源码，新增薄封装节点放在 `src/course_design`。
- 建图、导航、避障、识别、抓取都必须可单独 launch 调试。
- 新增的包需要撰写readme.md，说明如何 launch、调试、配置。

## 3. 可以复用的现有接口

底盘：

- `/controller/cmd_vel`，`geometry_msgs/msg/Twist`

SLAM/地图：

- `slam/launch/slam.launch.py`
- 地图目录 `src/slam/maps`
- 地图保存可复用 `/slam_toolbox/save_map`

导航：

- `navigation/launch/navigation.launch.py`
- `nav2_simple_commander.robot_navigator.BasicNavigator`
- 地图路径格式：`src/slam/maps/<map_name>.yaml`

避障：

- `app/launch/lidar_node.launch.py`
- `/lidar_app/enter`
- `/lidar_app/exit`
- `/lidar_app/set_running`
- `/lidar_app/set_param`

颜色识别：

- `/color_detect/set_param`
- `/color_detect/color_info`
- `/color_detect/image_result`

机械臂：

- 动作组目录 `/home/ubuntu/software/arm_pc/ActionGroups`
- 优先使用 `navigation_pick_init`、`navigation_pick`、`navigation_place`
- 三色放置优先映射 `place_left`、`place_center`、`place_right`

## 4. 安全边界

底盘：

- 避免底盘速度过大。
- 任何失败、超时、Ctrl+C、流程完成都必须发布零速度。
- 完整流程运行时必须有人看守。

机械臂：

- 抓取和放置前必须先停车。
- 动作组文件不存在时不得强行执行，必须日志提示并进入错误或 fallback。
- 完整流程结束时机械臂应回到安全姿态。

Nav2：

- 建图和最终搬运不要同时运行。
- 最终流程使用已保存地图。
- 导航超时必须 cancel task，并进入 `ERROR`。

## 5. 状态机规范

所有主流程节点必须用明确状态机，并在终端输出状态切换。

状态机要求：

- 每个状态进入时打印日志。
- 每个运动状态设置超时。
- `DONE` 和 `ERROR` 都必须停车。
- 错误日志包含：阶段、颜色、目标点、失败原因。

## 6. 终端输出规范

所有节点 `output='screen'`。

避免输出：

- 高频逐帧刷屏。
- 大段完整消息对象。
- 无意义的 `feedback` 重复日志。


## 7. Launch 规范

每个功能都要有独立 launch：

launch 编写要求：
- 不重复启动会冲突的硬件节点。
- 最终完整 launch 不启动 SLAM 建图。
- 建图 launch 不启动最终 workflow。

## 8. 配置规范

所有现场可调项写入 `src/course_design/config/course_design.yaml`：

- `map_name`
- `waypoints`
- 避障阈值、扫描角、速度
- 目标颜色顺序
- 视觉对准目标像素、容差、增益、速度上限
- Nav2、识别、对准、抓取、放置超时
- 机械臂动作组映射

代码中只能保留安全默认值，不硬编码现场坐标。

## 9. 构建与检查

基础构建（避免CPU负载过高，一定要单核编译）：

```bash
cd /home/ubuntu/ros2_ws
colcon build --packages-select course_design --parallel-workers 1
source install/setup.bash
```

手动停车：

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{}"
```

## 10. 调试顺序

必须按低耦合到高耦合推进：

1. 建图保存
2. 导航到命名点
3. 导航行为树
4. 颜色识别
5. 机械臂抓放
6. 单色搬运
7. 三色完整搬运

前一层不稳定时，不进入后一层。

## 11. 与 plan.md 的关系

- `plan.md` 是任务拆解和代码设计。
- `rules.md` 是实施约束和安全边界。
- 若二者冲突，以达成plan的要求为准。
