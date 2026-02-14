# inspection_sim

`inspection_sim` 是 inspection-robot 的**无实物联调/算法验证支撑包**：在没有 AGV/Elfin5 真机的情况下，提供一套“接口级仿真（fake drivers）”，让系统能跑通：

- `task_coordinator` 的状态机推进与到位门控
- `inspection_gateway` 的状态流订阅与控制面服务调用
- `path_planner` 的输入/输出数据流（先可用假规划器占位，后续替换为真实联合优化）
- HMI 的导航底图/状态监测（`GetNavMap` + `SubscribeSystemState`）

本包**不依赖 Gazebo**；目标是用更低成本复现“端到端效果”，并为论文实验提供可重复的对比环境。

## 快速开始（规划）

启动仿真系统（无硬件）：

```bash
ros2 launch inspection_sim sim_system.launch.py
```

如需同时启动 `inspection_gateway`（用于 HMI/gRPC 联调）：

```bash
ros2 launch inspection_sim sim_system.launch.py with_gateway:=true grpc_port:=50051
```

执行任务（ROS2 侧）：

```bash
ros2 service call /inspection/start inspection_interface/srv/StartInspection "{legacy_task_id: 0, plan_id: \"sim\", inspection_type: \"\", dry_run: false}"
```

观察状态：

```bash
ros2 topic echo /inspection/state
```

## 设计目标

- **接口对齐**：尽量复用真实驱动包的 Public ROS API（topic/service 名称与语义一致），让上层模块无需改代码即可切换“真机/仿真”。
- **可重复**：仿真过程可参数化（速度、延迟、噪声、固定 seed），便于做消融实验。
- **最小闭环**：优先满足 `task_coordinator` 的推进条件（`detect/optimize/detect_defect` 三个 Trigger + AGV/Arm status 门控）。

## 仿真范围（V1）

### 1) Fake AGV（替代 `agv_driver`）

目标：让系统在 `map` 坐标系下具备可控的“移动到底盘目标点”能力，并产出 `AgvStatus` 供联锁。

对外接口（对齐 `agv_driver`）：

- namespace：`/inspection/agv`
- 订阅：
  - `goal_pose` (`geometry_msgs/msg/PoseStamped`)
  - `cmd_vel` (`geometry_msgs/msg/Twist`)（可选）
- 发布：
  - `status` (`inspection_interface/msg/AgvStatus`)
  - `current_pose` (`geometry_msgs/msg/PoseStamped`)
  - `odom` (`nav_msgs/msg/Odometry`)（可选）
  - TF（可选）：`map -> base_link`
- 服务：
  - `get_nav_map` (`inspection_interface/srv/GetNavMap`)：返回可渲染的底图元信息与（可选）底图 PNG/JPEG

到位门控语义（必须满足 `task_coordinator`）：

```text
agv_ready = connected && arrived && stopped && (error_code == "OK")
```

### 2) Fake Arm（替代 `arm_driver`）

目标：持续发布 `ArmStatus` 与 `/joint_states`，使 `task_coordinator` 能通过 arrived 门控推进。

对外接口（对齐 `arm_driver`）：

- namespace：`/inspection/arm`
- 订阅：
  - `joint_cmd` (`sensor_msgs/msg/JointState`)（可选：接入 `arm_controller` 时使用）
- 发布：
  - `/joint_states` (`sensor_msgs/msg/JointState`)（全局话题）
  - `status` (`inspection_interface/msg/ArmStatus`)

默认关节名（Elfin5，来自 URDF）：

```text
elfin_joint1 ... elfin_joint6
```

### 3) Fake Perception / Planning / Defect（替代骨架节点依赖）

`task_coordinator` 当前依赖三个 Trigger 服务推进状态机：

- `/inspection/perception/detect` (`std_srvs/srv/Trigger`)
  - 建议同时发布固定的 `detected_pose`：`/inspection/perception/detected_pose`
- `/inspection/planning/optimize` (`std_srvs/srv/Trigger`)
  - 必须发布 `PoseArray`：`/inspection/planning/path`（至少 1 个 waypoint，否则无法进入 EXECUTING）
- `/inspection/perception/detect_defect` (`std_srvs/srv/Trigger`)
  - V1 可只返回 success；后续可发布 `inspection_interface/msg/DefectInfo` 以对齐事件/结果链路

说明：
- 真实联合优化算法建议落在 `path_planner` 的 `PlannerCore`（见 `src/path_planner/CLAUDE.md`），`inspection_sim` 只负责“仿真输入与执行环境”。

## 坐标与 TF（V1 约定）

- 所有位姿默认使用 `frame_id="map"`（单位：米/弧度）
- fake agv 建议发布 `map -> base_link` TF（用于 RViz/HMI 对齐）
- 机械臂安装外参 `base_link -> arm_base` 暂作为静态 TF（参数配置）；后续与真实标定统一

## 验收标准（P0）

- 在无硬件条件下：
  - `ros2 service call /inspection/start ...`（或通过 gRPC `StartInspection`）能启动任务
  - 状态机能推进：IDLE -> LOCALIZING -> PLANNING -> EXECUTING -> COMPLETED
  - `/inspection/state` 与 gRPC `SubscribeSystemState` 能稳定输出状态
