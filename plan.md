# ROS2 小车课程设计执行计划

## 1. 课程设计分层目标

课设分为两个层级：最低可完成目标和最终完整工作流。最低目标先证明小车具备移动机器人基础能力，最终工作流再叠加红、绿、蓝三色物块分类和机械臂搬运。

最低可完成目标：

1. 建图：启动底盘、雷达、SLAM，完成场地地图构建并保存。
2. 导航到点：加载已保存地图，给定目标点，小车通过 Nav2 自主导航到点。
3. 避障：在导航过程中能够绕开先验地图中的静态障碍物和不在地图中的动态障碍物。
4. 决策树：能够按照预先设计完成导航到目标点、往返两个点之间进行巡逻等任务。
5. 调试可分离：建图、导航、决策都能独立 launch 启动，并在终端输出关键状态。

最终完整工作流：

1. 遥控小车在场地中进行建图。
2. 在已建好的地图上启动导航。
3. 小车导航到在地图上标记好的取货区。
4. 摄像头识别红、绿、蓝物块。
5. 按目标颜色或固定顺序执行视觉对准、抓取。
6. 将物块按颜色搬运到目标位置分类放置。
6. 依次完成三色物块搬运，最后回到起点。
7. 每个子功能能单独 launch 调试，最终能用一个 launch 启动完整流程。

## 2. 当前环境可复用能力

已确认可直接复用：

- 建图：`src/slam/launch/slam.launch.py`
  - 支持 `slam_method:=slam_toolbox|cartographer|gmapping|rrt_explorer`
  - 默认建议使用 `slam_toolbox`
  - 会包含底盘、雷达等基础 robot launch
- 地图保存：`src/slam/slam/map_save.py`
  - 提供 `/slam_toolbox/save_map` 服务
  - 默认保存到 `src/slam/maps/map_01`
- 导航：`src/navigation/launch/navigation.launch.py`
  - 加载 `src/slam/maps/<map>.yaml`
  - 使用 Nav2，支持 `use_teb`
- 导航 API：`nav2_simple_commander.robot_navigator.BasicNavigator`
  - 当前环境可导入
  - 可用于新节点实现“导航到命名点”
- 导航避障：`src/navigation/config/nav2_params.yaml`
  - global/local costmap 已使用 `/scan` 作为障碍物观测源
  - local costmap 使用 `voxel_layer + inflation_layer`
  - global costmap 使用 `static_layer + obstacle_layer + inflation_layer`
  - 静态障碍主要来自已保存地图，动态障碍主要来自导航时的激光雷达 costmap 更新
- 控制器参数：`src/navigation/config/nav2_controller_teb.yaml` 和 `src/navigation/config/nav2_controller_dwb.yaml`
  - `navigation.launch.py` 支持 `use_teb`
  - 默认建议先用 TEB，现场可通过 YAML 调整速度、膨胀距离、障碍距离、目标容差等参数
- 颜色识别：`src/example/example/color_detect/color_detect_node.py`
  - `/color_detect/set_param`
  - `/color_detect/color_info`
  - `/color_detect/image_result`
- 机械臂动作组：
  - 目录 `/home/ubuntu/software/arm_pc/ActionGroups`
  - 已有 `navigation_pick_init`、`navigation_pick`、`navigation_place`
  - 已有 `place_left`、`place_center`、`place_right`
  - 可映射为红、绿、蓝分类放置区域

## 3. 新增代码总体设计

新增 ROS2 Python 包：src/course_design/

## 4. 配置文件设计

新增 `src/course_design/config/course_design.yaml`，集中管理地图、导航目标点、决策任务、视觉和机械臂动作等。

现场调试时只改 YAML，不直接改代码。

## 5. 最低可完成目标实现

### 5.1 建图调试

新增 launch：`mapping.launch.py`

职责：

- Include `slam/launch/slam.launch.py`
- 默认参数：
  - `slam_method:=slam_toolbox`
  - `sim:=false`
  - `enable_save:=true`
- 启动 `map_status_node`，订阅或周期检查关键话题，输出建图状态。

`map_status_node` 终端输出：

- 当前使用 SLAM 方法
- `/map` 是否收到
- `/scan` 或 `/scan_raw` 是否收到
- 当前地图保存提示

验收：

- RViz 或地图话题能看到地图逐步生成。
- 能通过 `/slam_toolbox/save_map` 或地图保存命令保存地图。
- `src/slam/maps/<map_name>.yaml` 和 `.pgm` 存在。

### 5.2 导航到点调试

新增 launch：`course_nav.launch.py`

职责：

- Include `navigation/launch/navigation.launch.py`
- 使用 `map_name` 加载 `src/slam/maps/<map_name>.yaml`
- 启动 `waypoint_nav_node`

`waypoint_nav_node` 职责：

- 从 YAML 读取 `waypoints`
- 提供服务：
  - `~/go_home`
  - `~/go_pick_area`
  - `~/go_goal_red`
  - `~/go_goal_green`
  - `~/go_goal_blue`
- 使用 `BasicNavigator.goToPose()` 执行导航
- 输出 Nav2 active、目标点、ETA、最终结果

验收：

- 能导航到 `home`、`pick_area` 至少两个点。
- 终端能看到目标点坐标、导航中反馈、成功/失败状态。
- 失败时不崩溃，明确输出 `FAILED` 或 `CANCELED`。
- 在导航过程中能绕开地图中的静态障碍。
- 在局部路径上临时放置障碍物时，local costmap 能更新并触发绕行、等待或重新规划。

### 5.3 导航避障参数预留

避障作为 Nav2 导航能力的一部分验证，参数调整保留在已有 YAML 中。

职责：

- 保留并记录 Nav2 costmap 与控制器调参入口。
- 导航调试时关注以下已有参数文件：
  - `src/navigation/config/nav2_params.yaml`
  - `src/navigation/config/nav2_controller_teb.yaml`
  - `src/navigation/config/nav2_controller_dwb.yaml`
- 现场根据导航表现调整：
  - costmap `inflation_radius`
  - costmap `cost_scaling_factor`
  - obstacle layer/voxel layer 的 `obstacle_max_range`
  - TEB `min_obstacle_dist`
  - TEB `max_vel_x`、`max_vel_theta`
  - goal checker 容差
  - progress checker 超时

终端输出：

- `course_nav.launch.py` 和 `waypoint_nav_node` 输出当前地图、目标点、Nav2 状态、导航结果。
- RViz 查看 global/local costmap 和规划路径，观察避障效果。

验收：

- 静态障碍：目标点导航路径能绕开已建图障碍物。
- 动态障碍：导航过程中临时加入障碍物，小车能减速、等待、局部绕行或重新规划。
- 避障表现不理想时，调整 Nav2 YAML。

### 5.4 决策树调试

新增 launch：`behavior.launch.py`

职责：

- Include `course_nav.launch.py`。
- 启动 `behavior_node`。
- `behavior_node` 复用公共导航工具函数和 `BasicNavigator`，自己执行预设任务树；不要依赖 `waypoint_nav_node` 已经启动。

支持任务：

- `go_named_point`：导航到指定命名点。
- `return_home`：返回起点。
- `patrol`：在两个命名点之间往返巡逻，默认 `pick_area <-> goal_green`，可指定次数，默认3次。


终端输出：

- 当前任务名
- 当前步骤编号
- 当前目标点
- 巡逻轮次
- 每次导航结果
- 任务完成或失败原因

验收：

- 能单独启动决策 launch。
- 能执行一次导航到目标点任务。
- 能在两个点之间完成指定次数往返巡逻。
- 任一步导航失败时，决策树进入 `ERROR` 并输出失败步骤。
