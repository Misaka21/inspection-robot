# task_coordinator/CLAUDE.md

本文件约束 `task_coordinator` 的架构与数据流。

> **方向变更（2026-04-23）**：从"巡检八点位循环"改为"复合机器人抓取 pipeline"。原 `MOVING_TO_STATION → ARM_PRESET → DEPTH_ADJUST → CAPTURING` 状态机已废弃；原 YAML 格式（`inspection_stations`）已不适用。详情见 `TODO.md`、`docs/ARCHITECTURE.md`。
> 旧的 8 点位 demo 脚本（`scripts/demo_inspection.py`）暂时保留用于回溯，但系统主路径不再使用。

## 1. 包职责与边界

负责：
- 抓取任务状态机（IDLE → NAV_TO_PICK → ARM_TO_OBSERVE → CAPTURE_RGBD → PERCEIVE → TRANSFORM_IK → PRE_GRASP → GRASP → CLOSE_GRIPPER → LIFT → NAV_TO_PLACE → PLACE → OPEN_GRIPPER → HOME → IDLE）
- YAML 配置驱动的任务模板执行（pick/place 点位、观察位、目标类别）
- 联锁门控（AGV 到位 → 允许观察 → 感知成功 → IK 通过 → 机械臂动作 → 夹爪控制）
- 感知→坐标变换→可达性筛选的串联调度
- 发布系统状态快照 `SystemState`（供网关/HMI 订阅）
- 发布结构化抓取事件 `GraspEvent`（供网关推 WebSocket）【P2 实现】

不负责：
- 硬件通信（drivers）
- MoveIt 规划算法本身（`arm_controller`）
- 抓取位姿生成算法（`grasp_perception`）
- 气动电磁阀 DO 控制（`gripper_driver` + `dio_driver`）
- 对外 REST/WS API（`inspection_gateway`）

## 2. Public ROS API

默认命名空间：`/inspection`

发布：
- `state` (`inspection_interface/msg/SystemState`) — 实时状态（phase + agv/arm/gripper status + progress）
- `events` (`inspection_interface/msg/GraspEvent`) — 结构化事件【P2】

订阅：
- `agv/status` (`inspection_interface/msg/AgvStatus`) → AGV 到位门控
- `arm/status` (`inspection_interface/msg/ArmStatus`) → 机械臂状态
- `gripper/status` (`inspection_interface/msg/GripperStatus`) → 夹爪状态【新】
- `realsense/d435/color/image_raw`、`.../aligned_depth_to_color/image_raw`、`.../color/camera_info`（可选：用于抓拍落盘）

发布（对 AGV）：
- `agv/goal_pose` (`geometry_msgs/msg/PoseStamped`) → AGV 导航目标（坐标）
- `agv/goal_station` (`std_msgs/msg/String`) → AGV 站点名称导航（厂商预设路径）

服务客户端：
- `arm_control/move_to_pose` (`inspection_interface/srv/MoveToPose`) → 包含 `planner_id` 字段，支持 OMPL / PILZ_LIN
- `arm_control/move_to_joints` (`inspection_interface/srv/MoveToJoints`) → 观察位、HOME
- `grasp/perceive_grasp` (`inspection_interface/srv/PerceiveGrasp`) → 感知抓取候选【新】
- `gripper/open` / `gripper/close` (`std_srvs/srv/Trigger`)【新】

服务（对外）：
- `start` (`inspection_interface/srv/StartGrasp`【新】，或扩充原 `StartInspection.srv`) / `stop` / `pause` / `resume` / `get_status`

参数：
- `grasp_tasks_file` (string) → YAML 抓取任务配置文件路径
- `default_task_name` (string) → 不传 task_name 时使用的默认模板
- `agv_timeout_sec` (double, 60.0)
- `arm_timeout_sec` (double, 30.0)
- `perceive_timeout_sec` (double, 10.0) — 感知服务调用超时
- `gripper_timeout_sec` (double, 2.0) — 夹爪 open/close 后等待稳定
- `max_perceive_retries` (int, 3) — PERCEIVE 无目标时重拍次数
- `max_grasp_retries` (int, 2) — GRASP 后抬起发现没夹到时重试次数
- `approach_offset_m` (double, 0.10) — PRE_GRASP 距抓取点高度（也可由 YAML 覆盖）

## 3. 状态机

```
IDLE
 └─ NAV_TO_PICK          (use_agv=false 时跳过)
     └─ ARM_TO_OBSERVE
         └─ CAPTURE_RGBD
             └─ PERCEIVE
                 └─ TRANSFORM_IK
                     └─ PRE_GRASP
                         └─ GRASP
                             └─ CLOSE_GRIPPER
                                 └─ LIFT
                                     ├─ (检测失败) → RECOVERY
                                     └─ NAV_TO_PLACE (可选)
                                         └─ PLACE
                                             └─ OPEN_GRIPPER
                                                 └─ HOME
                                                     └─ IDLE

任一阶段 → PAUSED / STOPPED / FAILED
RECOVERY: 根据失败类型选择重拍/重规划/终止
```

### 阶段说明

| 阶段 | 动作 | 成功条件 | 失败策略 |
|------|------|---------|----------|
| NAV_TO_PICK | 发 goal_pose / goal_station 给 AGV | `agv_status.arrived && stopped && error_code=="OK"` | 超时→FAILED |
| ARM_TO_OBSERVE | 调 `move_to_joints` | 服务 success=true | 超时→FAILED |
| CAPTURE_RGBD | 缓存最新同步 RGBD | 三者 stamp 差 < 50ms 且都非空 | 重取 3 次→FAILED |
| PERCEIVE | 调 `perceive_grasp` | `len(candidates) >= 1` | 最多 max_perceive_retries 次重拍 |
| TRANSFORM_IK | tf2 变换 + `move_to_pose(plan_only=true)` 筛选可达候选 | 至少 1 个候选 IK 通过 | 换观察位→再 PERCEIVE |
| PRE_GRASP | `move_to_pose` (PILZ_LIN) | motion_status=done | FAILED |
| GRASP | `move_to_pose` (PILZ_LIN, 下压到抓取点) | motion_status=done | FAILED |
| CLOSE_GRIPPER | 调 `gripper/close`，等 gripper_timeout_sec | gripper_status.is_closed && !moving | FAILED |
| LIFT | `move_to_pose` (PILZ_LIN, 上升) | motion_status=done | FAILED |
| (抓取后检测) | 复核是否真夹到（可选：DI 压力反馈 / 二次观察） | 判定成功 | 重拍重抓 max_grasp_retries 次 |
| NAV_TO_PLACE | 同 NAV_TO_PICK | 同上 | |
| PLACE | 移到放置点上方→下压→OPEN_GRIPPER | 序列完成 | FAILED |
| OPEN_GRIPPER | 调 `gripper/open`，等 gripper_timeout_sec | gripper_status.is_closed==false | FAILED |
| HOME | `move_to_joints(home)` | motion_status=done | 单独日志警告，不阻塞下一次任务 |

## 4. YAML 配置格式

`config/grasp_tasks.yaml` 示例：

```yaml
tasks:
  demo_pick_screw:
    object_class: "screw_m8"
    pick:
      use_agv: true
      agv_station: "PICK_01"          # 或 agv_pose: {x, y, yaw}
      arm_observe_joints: [0.0, -0.5, 1.2, 0.0, 0.8, 0.0]
      approach_offset_m: 0.10
      velocity_scaling: 0.2
    place:
      use_agv: true
      agv_station: "PLACE_01"
      arm_place_joints: [0.0, 0.3, 0.8, 0.0, 1.1, 0.0]
      release_offset_m: 0.05
    recovery:
      max_perceive_retries: 3
      max_grasp_retries: 2

  # 可扩展更多任务模板
  demo_pick_bolt:
    object_class: "bolt_m10"
    pick: ...
    place: ...
```

## 5. 数据流

```mermaid
flowchart TB
  Start["/inspection/start(srv)"] --> CO["task_coordinator"]
  YAML["grasp_tasks.yaml"] --> CO

  CO -->|goal_pose / goal_station| AGV["agv_driver"]
  AGV -->|status (arrived/stopped)| CO

  CO -->|MoveToJoints (observe/home)| ARMC["arm_controller"]
  CO -->|MoveToPose (PILZ_LIN pre_grasp/grasp/lift)| ARMC

  RS["realsense_driver"] -->|RGBD + camera_info| GP["grasp_perception"]
  CO -->|PerceiveGrasp| GP
  GP -->|GraspPose[] (camera_frame)| CO

  CO -->|tf2 lookup| TF["TF tree"]

  CO -->|gripper/open / gripper/close| GR["gripper_driver"]
  GR -->|dio/set_output| DIO["dio_driver"]

  CO -->|/inspection/state| GW["inspection_gateway"]
  CO -->|/inspection/events| GW
```

## 6. 推荐内部架构（避免所有逻辑堆在 Node 回调）

建议拆成 3 层：

1. `CoordinatorCore`（无 ROS 依赖）
   - 状态机 + 阶段规则 + 超时/重试计数
   - 纯函数式接口：输入 `Inputs`（status/感知结果/时间），输出 `Actions`（要触发的下一步）
   - 可单测
2. `InterlockPolicy`
   - 把"门控"规则集中：`agv_ready()`、`arm_ready()`、`gripper_ready()`、`grasp_candidates_valid()`
   - 避免散落在状态机分支里
3. `RosAdapter(Node)`
   - ROS pub/sub/srv/timer、tf2 lookup
   - 维护 Inputs/Actions 的线程安全封装
   - 把 Action 翻译成 ROS 调用

约束：
- ROS 回调里不做长时间阻塞（service async + future + timer tick）
- 感知/规划调用放 worker 线程或独立 executor

## 7. 文档与 TODO 维护（必须）

- 修改 public ROS API（topic/service/参数）或状态机推进规则时，必须同步更新：本文件、`docs/ARCHITECTURE.md`、仓库根 `TODO.md`
- 任何"阶段性实现/临时逻辑"必须在 `TODO.md` 留痕
