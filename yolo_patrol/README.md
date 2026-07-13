# yolo_patrol：基于 YOLO 的巡逻异常检测任务

`yolo_patrol` 是一个独立 ROS 2 Python 功能包，用于在原有小车巡逻任务基础上加入 YOLO 目标检测。它不直接修改 `course_design` 和 `example/yolo_detect` 源码，方便后续单独调试、复制和回退。

## 1. 功能目标

本任务面向实验室、工业车间、仓储区域等日常巡检场景。小车不只是按照固定点位移动，还会在巡检点停车观察，识别人员、瓶子、手机或遥控器等目标，并根据不同类别执行不同策略。

核心流程：

```text
读取巡检点 → Nav2导航 → 到点停车 → YOLO检测 → 多帧判断 → 继续巡逻/告警停车/等待人工确认
```

## 2. 与已有任务的区别

| 任务 | 主要作用 | 决策来源 |
|---|---|---|
| patrol | 在两个或多个命名点之间巡逻 | 固定点位顺序 |
| qrcode_target | 根据二维码内容选择目标点 | 二维码 |
| yolo_patrol | 巡逻到点后检测环境目标并判断风险 | YOLO目标检测 |

`yolo_patrol` 不做“识别到目标就导航到某处”的换皮逻辑，而是把识别结果接入巡逻状态机。

## 3. 复用的已有能力

本包主要复用：

- `course_design` 中的地图、命名点、Nav2导航、零速度停车逻辑；
- `example/yolo_detect` 中的 YOLO 推理节点；
- `interfaces/msg/ObjectsInfo.msg` 中的目标检测结果格式；
- NoMachine 中的 RViz 与图像显示环境。

## 4. 巡检点配置

巡检点建议写在：

```bash
/home/ubuntu/ros2_ws/src/yolo_patrol/config/yolo_patrol_course.yaml
```

每个点需要记录：

```yaml
x: 地图坐标x
y: 地图坐标y
yaw_deg: 小车到达后朝向角
```

示例：

```yaml
waypoints:
  inspect_1:
    x: 1.0
    y: 0.0
    yaw_deg: 0.0
```

其中：

- `x、y` 决定小车去哪里；
- `yaw_deg` 决定小车到点后摄像头朝向哪里；
- 如果到点后看不到目标，通常优先检查 `yaw_deg` 和点位距离。

## 5. 检测与处理策略

第一版建议使用 COCO 通用模型 `yolo26n`，不急着训练新模型。

当前测试结果适合采用：

| 类别 | 处理策略 | 说明 |
|---|---|---|
| person | safety_stop | 人员出现，安全停车，等待人工确认 |
| bottle | log_continue | 普通遗留物，记录后继续巡逻 |
| cell phone | alert_stop | 疑似电子设备或遗留物，告警停车 |
| remote | alert_stop | 与 cell phone 合并处理，告警停车 |

注意：手机和遥控器在模型中可能跳变，因此任务层面不强行区分，而是统一归为“疑似电子设备或遗留物”。

## 6. 移动中是否检测

本包默认采用折中方案：

```text
移动中轻量监听，只记录候选目标；
到达巡检点停车后，才正式检测并作出任务决策。
```

这样设计的原因：

- 移动中图像容易模糊；
- 单帧误检容易导致频繁停车；
- 到点后检测更稳定；
- 移动中候选目标可以帮助判断“到点后没识别到”是否可能是漏检。

可能状态：

| 状态 | 含义 |
|---|---|
| confirmed | 多帧确认目标 |
| no_target | YOLO正常，但没有关注目标 |
| no_yolo_data | 没有收到 YOLO 消息 |
| missed_after_moving | 移动中疑似看到，到点后未确认 |
| unstable | 有目标但帧数或置信度不足 |

## 7. 快速启动

编译：

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select yolo_patrol
source install/setup.zsh
```

完整启动：

```bash
ros2 launch yolo_patrol yolo_patrol.launch.py
```

在 RViz 设置初始位姿后，启动任务：

```bash
ros2 topic pub --once /yolo_patrol/start std_msgs/msg/Bool "{data: true}"
```

若停车等待人工确认，发送：

```bash
ros2 topic pub --once /yolo_patrol/reset std_msgs/msg/Bool "{data: true}"
```

更多现场操作见：

```text
OPERATIONS.md
```

## 8. 后续模型改进

如果 COCO 模型对现场目标识别不稳定，可以考虑：

1. 调整参数：降低或提高 `min_score`，增加 `observe_time_sec`，增加 `stable_min_count`；
2. 规范目标摆放：瓶子竖立、目标在画面中央、减少反光；
3. 重新采集课程场地数据并训练自定义 YOLO 模型；
4. 将自训练模型导出为 engine 后替换运行模型。

第一版优先目标不是追求检测精度完美，而是先跑通“巡逻—检测—判断—处置”的闭环。
