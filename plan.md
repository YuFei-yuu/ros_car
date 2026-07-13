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

## 6. 单色自主搬运工作流

本阶段在第 5 节的导航能力基础上完成一个可配置颜色的完整闭环。一次任务只搬运一种颜色，默认红色：初始化 -> 导航到 `pick_area` -> 视觉对准并抓取目标颜色物块 -> 导航到对应目标区 -> 放置 -> 返回 `home`。

不修改厂家 `example`、`slam`、`navigation`、`app` 包；所有新增封装仍放在 `src/course_design`。颜色识别复用厂家 `color_detect` 可执行节点和既有 ROS 接口，机械臂复用已有动作组。

### 6.1 工作流范围与默认策略

- 目标颜色由 `course_design.yaml` 的 `transport.target_color` 配置，默认 `red`。
- 颜色到导航点使用 `transport.goal_by_color` 映射；默认 `red -> goal_red`，同时预留 `green -> goal_green`、`blue -> goal_blue`。
- 色块默认使用 `rect` 检测。
- 抓取前只识别配置的目标颜色；存在多个同色块时选择检测尺寸最大的目标。
- 视觉对准稳定后运行 `navigation_pick`；动作组正常运行完成即视为抓取成功。
- 到达目标点后统一运行 `navigation_place`。
- 放置后自动导航回 `home`，再让机械臂回到 `navigation_pick_init` 安全姿态并结束。

### 6.2 配置扩展

扩展 `src/course_design/config/course_design.yaml`：

- `transport`：目标颜色、颜色到目标点映射、初始位姿要求、启动/取消话题。
- `vision`：图像识别类型、目标像素坐标、水平/前后容差、稳定帧数、线速度和角速度增益、速度上限、检测丢失和对准超时。
- `arm`：动作组目录、`pick_ready`、`pick`、`place`、`safe` 动作名，以及每个动作的超时。
- 继续复用现有 `navigation` 的导航超时、反馈周期和停车话题。

不在代码中固化地图坐标、ROI、速度方向或机械臂动作组名称。线速度增益允许为正或负，以适配相机像素 y 方向与实际前进方向的现场标定结果。

### 6.3 取放节点调试

新增节点：`pick_place_node.py`。

职责：

- 订阅 `/color_detect/color_info`，维护最新的目标颜色检测结果。
- 调用 `/color_detect/set_param` 配置单一目标颜色和检测类型；使用 `/controller/cmd_vel` 做低速视觉对准。
- 连续达到配置的稳定帧数后发布零速度，执行抓取动作组；放置请求执行放置动作组。
- 在执行前检查动作组 `.d6a` 文件是否存在；机械臂动作在工作线程运行并受超时保护。
- 任意检测超时、目标丢失、动作超时或动作组缺失都发布零速度、记录失败原因并返回失败。

提供服务，均复用现有接口类型：

- `~/set_target_color`：`interfaces/srv/SetString`，设置并校验目标颜色。
- `~/pick`：`std_srvs/srv/Trigger`，执行视觉对准和抓取。
- `~/place`：`std_srvs/srv/Trigger`，执行 `navigation_place`。
- `~/stop`：`std_srvs/srv/Trigger`，停止底盘并中止当前取放流程。

新增独立 launch：`color_pick.launch.py`。

- 启动一次底盘、相机、舵机基础硬件，启动厂家 `color_detect` 节点和 `pick_place_node`。
- 不 include 厂家 `color_detect_node.launch.py`，避免重复启动深度相机。
- 用于单独调试颜色识别、视觉对准、抓取和放置，不能启动 Nav2 或 SLAM。

验收：

- 可通过 `set_target_color` 设置 `red`，并在 `/color_detect/color_info` 观察目标颜色的像素坐标。
- 物块进入配置 ROI 后，小车低速对准并停车，机械臂执行 `navigation_pick`。
- 调用 `place` 可执行 `navigation_place`；检测或动作失败时底盘保持停止。

### 6.4 完整搬运状态机

新增节点：`transport_workflow_node.py`。

节点独立创建 `BasicNavigator`，直接复用 `navigation_utils.run_navigation()`，不依赖 `waypoint_nav_node` 的服务，也不与它并发发送导航目标。

状态机固定为：

1. `WAIT_INITIAL_POSE`：等待 RViz 发布 `/initialpose`，并等待人工启动命令。
2. `INITIALIZE`：发布零速度，校验 YAML、颜色目标点、取放服务和动作组文件，运行 `navigation_pick_init`。
3. `GO_PICK_AREA`：导航至 `pick_area`，超时后取消 Nav2 任务。
4. `PICK`：设置目标颜色并调用 `pick_place_node` 的抓取服务。
5. `GO_GOAL`：按 `goal_by_color` 导航至目标点，例如 `goal_red`。
6. `PLACE`：确认底盘停止后调用放置服务。
7. `RETURN_HOME`：导航至 `home`。
8. `DONE`：发布零速度，运行安全动作，输出完成日志。
9. `ERROR`：取消 Nav2、发布零速度、尽力运行安全动作，输出阶段、颜色、目标点和失败原因。

对外接口：

- `~/start`、`~/cancel`：`std_srvs/srv/Trigger`。
- 配置的启动/取消 `std_msgs/msg/Bool` 话题。
- `/transport_workflow/state`：`std_msgs/msg/String`，发布当前状态，便于终端和录屏验证。

新增 launch：`transport.launch.py`。

- Include `navigation/launch/navigation.launch.py`，使用已保存地图，绝不启动 SLAM。
- 启动厂家 `color_detect` 可执行节点、`pick_place_node`、`transport_workflow_node` 和 RViz。
- 不启动二维码识别、`behavior_node` 或任何会重复启动相机、底盘、舵机的厂家 launch。
- 所有节点使用 `output='screen'`；启动后默认不移动，必须先在 RViz 设置初始位姿并显式调用启动服务。

### 6.5 调试与验收顺序

1. 继续使用 `course_nav.launch.py` 验证 `home`、`pick_area` 和当前颜色目标点的导航、停车及避障。
2. 启动 `color_pick.launch.py`，先标定目标像素、容差、增益和速度上限，再验证单独抓取、单独放置。
3. 启动 `transport.launch.py`，在 RViz 设置初始位姿，调用 `transport_workflow_node` 的启动服务。
4. 检查状态按 `INITIALIZE -> GO_PICK_AREA -> PICK -> GO_GOAL -> PLACE -> RETURN_HOME -> DONE` 转换。
5. 分别验证未检测到色块、目标丢失、动作组缺失、动作超时、导航失败/超时和人工取消；每种情况都必须停止底盘并进入 `ERROR` 或取消状态。
6. 为配置校验、颜色到目标映射、速度限幅和稳定帧判定添加不依赖硬件的单元测试。
7. 使用单核命令构建并检查：

```bash
cd /home/ubuntu/ros2_ws
colcon build --packages-select course_design --parallel-workers 1
source install/setup.bash
```