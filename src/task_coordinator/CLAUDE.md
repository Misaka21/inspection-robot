# task_coordinator/CLAUDE.md

本文件约束 `task_coordinator` 的架构与数据流。当前主线为**大型工件视觉检测/巡检**，不是抓取 pipeline。

## 1. 包职责与边界

负责：

- 巡检任务状态机：`IDLE -> MOVING_TO_STATION -> ARM_PRESET -> DEPTH_ADJUST -> CAPTURING -> 下一站/COMPLETED`
- YAML 配置驱动的站位序列执行。
- 联锁门控：AGV 到位后才允许机械臂动作；机械臂到预设位后才允许深度微调；工作距满足容差后才拍照检测。
- RealSense 深度图工作距估计与微调调度。
- 触发海康工业相机拍照。
- 触发 `defect_detector` 并订阅缺陷结果。
- 发布系统状态快照 `SystemState`。

不负责：

- AGV、机械臂、相机硬件通信。
- MoveIt2 规划细节。
- 缺陷检测模型推理细节。
- 网页/REST/WS 接口。
- fake driver 仿真。

## 2. Public ROS API

默认命名空间：`/inspection`

发布：

- `state` (`inspection_interface/msg/SystemState`)
- `agv/goal_pose` (`geometry_msgs/msg/PoseStamped`)
- `agv/goal_station` (`std_msgs/msg/String`，可选)

订阅：

- `agv/status` (`inspection_interface/msg/AgvStatus`)
- `arm/status` (`inspection_interface/msg/ArmStatus`)
- `/inspection/realsense/d435/aligned_depth_to_color/image_raw`
- `/inspection/realsense/d435/color/camera_info`（增强微调需要）
- `perception/result` (`inspection_interface/msg/DefectInfo`)

服务客户端：

- `arm_control/move_to_joints` (`inspection_interface/srv/MoveToJoints`)
- `arm_control/move_to_pose` (`inspection_interface/srv/MoveToPose`)
- `hikvision/trigger_capture` (`std_srvs/srv/Trigger`)
- `perception/detect_defect` (`std_srvs/srv/Trigger`)

服务：

- `start` / `stop` / `pause` / `resume` / `get_status`

参数：

- `stations_file`：巡检站位 YAML。
- `agv_timeout_sec`
- `arm_timeout_sec`
- `depth_adjust_timeout_sec`
- `detection_timeout_sec`
- `max_depth_retries`
- `depth_roi_center_x` / `depth_roi_center_y` / `depth_roi_width` / `depth_roi_height`（建议新增）
- `depth_kp` / `depth_max_step`（建议新增）

## 3. 状态机

```text
IDLE
  -> MOVING_TO_STATION
  -> ARM_PRESET
  -> DEPTH_ADJUST
  -> CAPTURING
  -> 下一站位 / COMPLETED
```

每个执行阶段都支持 `PAUSED`、`STOPPED`、`FAILED`。

| 阶段 | 动作 | 成功条件 |
|---|---|---|
| `MOVING_TO_STATION` | 发布 AGV 目标 | `agv_status.arrived && connected && stopped && error_code=="OK"` |
| `ARM_PRESET` | 调 `MoveToJoints` | 服务返回 success |
| `DEPTH_ADJUST` | 读 RealSense 深度，估计工作距，调 `MoveToPose` | 距离误差小于容差 |
| `CAPTURING` | 触发海康拍照，调用缺陷检测 | 收到检测结果或记录失败 |

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

## 5. 工作距微调约束

当前代码已有中心 ROI 深度中值测距骨架，但需要修正：

- `MoveToPose.target_pose` 按绝对位姿解释，不能直接把 `position.z = delta` 当相对位移。
- 应获取当前 TCP 位姿，并把深度误差转换到 planning frame 后叠加成新的绝对目标位姿。
- 增强算法应订阅 `CameraInfo`，把 ROI 深度反投影成局部点云，使用 PCA / RANSAC 估计非平面局部切平面。

## 6. CAPTURING 阶段约束

目标顺序：

```text
trigger_capture
  -> 等待 /inspection/hikvision/image_raw 新帧
  -> detect_defect
  -> 等待 perception/result
  -> 记录当前站位结果
```

不要只调用 `detect_defect` 而跳过相机触发。

## 7. 文档与 TODO 维护

修改 public ROS API、状态机阶段、站位配置格式或端到端数据流时，必须同步更新：

- 仓库根 `TODO.md`
- `docs/ARCHITECTURE.md`
- `docs/IMPLEMENTATION_STATUS.md`
- `docs/WORKSPACE_OVERVIEW.md`

