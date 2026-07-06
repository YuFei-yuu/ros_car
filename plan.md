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

新增 ROS2 Python 包：

```text
src/course_design/
  package.xml
  setup.py
  setup.cfg
  resource/course_design
  course_design/
    __init__.py
    common.py
    map_status_node.py
    waypoint_nav_node.py
    behavior_node.py
    color_debug_node.py
    align_debug_node.py
    arm_task_node.py
    single_color_transport_node.py
    sorting_workflow_node.py
  launch/
    mapping_debug.launch.py
    navigation_debug.launch.py
    behavior_debug.launch.py
    color_debug.launch.py
    align_debug.launch.py
    arm_debug.launch.py
    single_color_transport.launch.py
    full_sorting_workflow.launch.py
  config/
    course_design.yaml
```

注册 console scripts：

```text
map_status = course_design.map_status_node:main
waypoint_nav = course_design.waypoint_nav_node:main
behavior = course_design.behavior_node:main
color_debug = course_design.color_debug_node:main
align_debug = course_design.align_debug_node:main
arm_task = course_design.arm_task_node:main
single_color_transport = course_design.single_color_transport_node:main
sorting_workflow = course_design.sorting_workflow_node:main
```

## 4. 配置文件设计

新增 `src/course_design/config/course_design.yaml`，集中管理地图、导航目标点、决策任务、视觉和机械臂动作等。

建议初始内容：

```yaml
/**:
  ros__parameters:
    map_name: "map_01"
    map_frame: "map"
    base_frame: "base_footprint"

    waypoints:
      home: [0.0, 0.0, 0.0]
      pick_area: [1.0, 0.0, 0.0]
      place_red: [1.0, 0.6, 0.0]
      place_green: [1.0, 0.0, 0.0]
      place_blue: [1.0, -0.6, 0.0]

    navigation:
      use_teb: true
      nav_timeout_sec: 90.0
      controller_params_file: "nav2_params.yaml"
      controller_tuning_note: "Nav2静态/动态避障通过navigation/config中的costmap和controller YAML现场调整"

    behavior:
      default_task: "go_pick_area"
      patrol_points: ["home", "pick_area"]
      patrol_cycles: 3
      wait_at_point_sec: 2.0

    colors:
      target_colors: ["red", "green", "blue"]
      detect_type: "circle"

    align:
      target_x: 320
      target_y: 270
      x_tolerance: 25
      y_tolerance: 25
      stable_frames: 20
      max_linear_x: 0.10
      max_angular_z: 0.35
      linear_kp: 0.001
      angular_kp: 0.003
      timeout_sec: 8.0

    actions:
      init: "navigation_pick_init"
      pick: "navigation_pick"
      place_red: "place_left"
      place_green: "place_center"
      place_blue: "place_right"
      fallback_place: "navigation_place"

    workflow:
      run_order: ["red", "green", "blue"]
      detect_timeout_sec: 10.0
      nav_timeout_sec: 90.0
      pick_timeout_sec: 12.0
      place_timeout_sec: 12.0
      return_home: true
```

现场调试时只改 YAML，不直接改代码。

## 5. 最低可完成目标实现

### 5.1 建图调试

新增 launch：`mapping_debug.launch.py`

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

新增 launch：`navigation_debug.launch.py`

职责：

- Include `navigation/launch/navigation.launch.py`
- 使用 `map_name` 加载 `src/slam/maps/<map_name>.yaml`
- 启动 `waypoint_nav_node`

`waypoint_nav_node` 职责：

- 从 YAML 读取 `waypoints`
- 提供服务：
  - `~/go_home`
  - `~/go_pick_area`
  - `~/go_named_point`，如需要可使用 `interfaces/srv/SetString`
- 使用 `BasicNavigator.goToPose()` 执行导航
- 输出 Nav2 active、目标点、ETA、最终结果

验收：

- 能导航到 `home`、`pick_area` 至少两个点。
- 终端能看到目标点坐标、导航中反馈、成功/失败状态。
- 失败时不崩溃，明确输出 `FAILED` 或 `CANCELED`。
- 在导航过程中能绕开地图中的静态障碍。
- 在局部路径上临时放置障碍物时，local costmap 能更新并触发绕行、等待或重新规划。

### 5.3 导航避障参数预留

不再设计独立雷达避障 launch。避障作为 Nav2 导航能力的一部分验证，参数调整保留在已有 YAML 中。

职责：

- 保留并记录 Nav2 costmap 与控制器调参入口。
- 不新增 `obstacle_debug_node`，不调用 `/lidar_app/set_running`。
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

- `navigation_debug.launch.py` 和 `waypoint_nav_node` 输出当前地图、目标点、Nav2 状态、导航结果。
- 如需观察避障效果，使用 RViz 查看 global/local costmap 和规划路径。

验收：

- 静态障碍：目标点导航路径能绕开已建图障碍物。
- 动态障碍：导航过程中临时加入障碍物，小车能减速、等待、局部绕行或重新规划。
- 避障表现不理想时，只调整 Nav2 YAML，不新增独立雷达避障玩法。

### 5.4 决策树调试

新增 launch：`behavior_debug.launch.py`

职责：

- Include `navigation_debug.launch.py` 或直接 Include `navigation/launch/navigation.launch.py`。
- 启动 `behavior_node`。
- `behavior_node` 复用公共导航工具函数和 `BasicNavigator`，自己执行预设任务树；不要依赖 `waypoint_nav_node` 已经启动。

初始支持任务：

- `go_named_point`：导航到指定命名点。
- `go_pick_area`：导航到取货区。
- `return_home`：返回起点。
- `patrol`：在两个命名点之间往返巡逻，默认 `home <-> pick_area`。

建议状态机：

```text
IDLE
LOAD_TASK
NAV_TO_POINT
WAIT_AT_POINT
NEXT_STEP
DONE
ERROR
```

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

## 6. 最终完整工作流实现

### 6.1 单功能调试节点

保留以下可单独 launch 的调试节点：

- `color_debug_node`
  - 设置红、绿、蓝颜色识别
  - 输出识别到的颜色、中心点、半径
- `align_debug_node`
  - 只做视觉对准，不抓取
  - 发布 `/controller/cmd_vel`
  - 对准完成后停车
- `arm_task_node`
  - 只调机械臂动作组
  - 支持 `mode:=pick|place|init`
  - 支持 `target_color:=red|green|blue`
- `single_color_transport_node`
  - 单色闭环：导航到取货区 -> 识别 -> 对准 -> 抓取 -> 放置 -> 可选回起点

### 6.2 完整 workflow 节点

新增 `sorting_workflow_node`。

主状态机：

```text
INIT
NAV_TO_PICK
DETECT_COLOR
ALIGN_OBJECT
PICK_OBJECT
NAV_TO_PLACE
PLACE_OBJECT
NEXT_COLOR
RETURN_HOME
DONE
ERROR
```

执行策略：

- 按 `workflow.run_order` 处理颜色，默认 `red -> green -> blue`
- 每种颜色都导航到 `pick_area`
- 在取货区识别并对准目标颜色
- 抓取后导航到对应放置点：
  - `red -> place_red`
  - `green -> place_green`
  - `blue -> place_blue`
- 到达放置点后执行对应放置动作：
  - `red -> place_left`
  - `green -> place_center`
  - `blue -> place_right`
- 所有颜色完成后按参数决定是否回 `home`

终端必须输出：

- 当前阶段
- 当前颜色
- 当前导航目标点
- Nav2 ETA 或反馈摘要
- 识别目标中心点
- 对准误差
- 抓取/放置动作组名称
- 每个阶段成功或失败原因

## 7. Launch 文件规划

最低目标 launch：

- `mapping_debug.launch.py`
- `navigation_debug.launch.py`
- `behavior_debug.launch.py`

物块搬运调试 launch：

- `color_debug.launch.py`
- `align_debug.launch.py`
- `arm_debug.launch.py`
- `single_color_transport.launch.py`

最终完整 launch：

- `full_sorting_workflow.launch.py`

`full_sorting_workflow.launch.py` 启动：

1. `navigation/launch/navigation.launch.py`
2. `example` 的 `color_detect`
3. `sorting_workflow_node`

不建议在最终 launch 中同时启动建图。最终流程应使用已经保存好的地图。

## 8. 推荐实施顺序

1. 建立 `course_design` 包、配置文件、公共工具。
2. 实现 `mapping_debug.launch.py` 和 `map_status_node`。
3. 实现 `navigation_debug.launch.py` 和 `waypoint_nav_node`。
4. 保留 Nav2 避障参数入口，使用 `navigation_debug.launch.py` 验证静态/动态避障。
5. 实现 `behavior_debug.launch.py` 和 `behavior_node`。
6. 实现 `color_debug_node`。
7. 实现 `align_debug_node`。
8. 实现 `arm_task_node`。
9. 实现 `single_color_transport_node`。
10. 实现 `sorting_workflow_node` 和 `full_sorting_workflow.launch.py`。
11. 按最低目标、单色闭环、三色完整流程依次验收。

## 9. 验收标准

最低可完成目标：

- 能完成一次建图并保存地图文件。
- 能加载地图并导航到至少两个命名点。
- 导航过程中能绕开地图静态障碍，并能对临时动态障碍做出局部避让、等待或重新规划。
- 能单独启动决策树任务，完成导航到点和两点往返巡逻。
- 建图、导航、决策都有独立 launch 和清晰终端输出。

最终完整工作流：

- 能单独调试颜色识别、视觉对准、机械臂抓放、单色搬运。
- 能通过一个 launch 启动完整三色分类搬运。
- 流程失败时能停车并输出失败阶段。
- 流程完成时输出 `DONE`，底盘速度为 0，机械臂处于安全姿态。
