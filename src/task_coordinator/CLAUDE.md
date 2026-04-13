# task_coordinator/CLAUDE.md

本文件约束 `task_coordinator` 的架构与数据流。

## 1. 包职责与边界

负责：
- 巡检任务状态机（IDLE → MOVING_TO_STATION → ARM_PRESET → DEPTH_ADJUST → CAPTURING → 下一站/COMPLETED）
- YAML 配置驱动的站位序列执行
- 深度相机测距 + 机械臂微调（在 DEPTH_ADJUST 阶段）
- 联锁门控（AGV 到位 → 允许机械臂动作 → 深度微调 → 触发拍照/检测）
- 发布系统状态快照 `SystemState`（供网关/HMI 订阅）

不负责：
- 控制算法（drivers/controllers）
- 缺陷检测算法（`defect_detector`）
- 对外 REST/WS API（`inspection_gateway`）

## 2. Public ROS API

默认命名空间：`/inspection`

发布：
- `state` (`inspection_interface/msg/SystemState`)

订阅：
- `agv/status` (`inspection_interface/msg/AgvStatus`) → AGV 到位门控
- `arm/status` (`inspection_interface/msg/ArmStatus`) → 机械臂状态
- `/inspection/realsense/d435/aligned_depth_to_color/image_raw` (`sensor_msgs/msg/Image`) → 深度测距

发布（对 AGV）：
- `agv/goal_pose` (`geometry_msgs/msg/PoseStamped`) → AGV 导航目标（坐标）
- `agv/goal_station` (`std_msgs/msg/String`) → AGV 站点名称导航（RoboShop 预设路径）

服务客户端：
- `arm_control/move_to_joints` (`inspection_interface/srv/MoveToJoints`) → 机械臂预设位姿
- `arm_control/move_to_pose` (`inspection_interface/srv/MoveToPose`) → 深度微调
- `perception/detect_defect` (`std_srvs/srv/Trigger`) → 触发缺陷检测
- `/inspection/dio/set_output` (`inspection_interface/srv/SetDioOutput`) → 控制 DO 输出

服务（对外）：
- `start` / `stop` / `pause` / `resume` / `get_status`（`inspection_interface/srv/*`）

参数：
- `stations_file` (string) → YAML 站位配置文件路径
- `agv_timeout_sec` (double, 60.0) → AGV 到位超时
- `arm_timeout_sec` (double, 30.0) → 机械臂移动超时
- `detection_timeout_sec` (double, 10.0) → 缺陷检测超时
- `depth_adjust_timeout_sec` (double, 15.0) → 深度微调超时
- `max_depth_retries` (int, 3) → 深度微调最大重试次数

## 3. 状态机

```
IDLE → MOVING_TO_STATION → ARM_PRESET → DEPTH_ADJUST → CAPTURING → (下一站 or COMPLETED)
         ↑                                                    │
         └────────────────────────────────────────────────────┘
```

每个执行阶段都支持 → PAUSED / STOPPED / FAILED。

### 阶段说明

| 阶段 | 动作 | 成功条件 |
|------|------|---------|
| MOVING_TO_STATION | 发 goal_pose 给 AGV | agv_status.arrived && connected && stopped && error_code=="OK" |
| ARM_PRESET | 调 MoveToJoints 服务 | 服务返回 success=true |
| DEPTH_ADJUST | 读深度图 ROI 中值，计算偏移，调 MoveToPose | |delta| < tolerance 或超过 max_retries |
| CAPTURING | 调 detect_defect 服务 | 服务返回（成功或失败都继续） |

## 4. YAML 配置格式

```yaml
stations:
  - name: "station_1"
    agv_pose: {x: 1.0, y: 2.0, z: 0.0, yaw: 0.5}
    arm_joints: [0.0, -0.5, 1.2, 0.0, 0.8, 0.0]
    target_distance: 0.30
    distance_tolerance: 0.02
    adjust_axis: "z"
```

## 5. 数据流

```mermaid
flowchart TB
  Start["/inspection/start(srv)"] --> CO["task_coordinator"]
  YAML["inspection_stations.yaml"] --> CO

  CO -->|goal_pose| AGV["agv_driver"]
  AGV -->|status (arrived)| CO

  CO -->|MoveToJoints| ARM["arm_controller"]
  CO -->|MoveToPose (微调)| ARM

  RS["realsense_driver"] -->|depth image| CO
  CO -->|detect_defect| DET["defect_detector"]

  CO -->|/inspection/state| GW["inspection_gateway"]
```

## 6. 文档与 TODO 维护（必须）

- 修改 public ROS API（topic/service/参数）或状态机推进规则时，必须同步更新：本文件、`docs/ARCHITECTURE.md`、仓库根 `TODO.md`
- 任何"阶段性实现/临时逻辑"必须在 `TODO.md` 留痕
