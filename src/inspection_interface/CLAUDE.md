# inspection_interface/CLAUDE.md

本包是机器人端的"内部契约层"：所有跨包通信都应优先使用本包定义的 msg/srv。

> **方向说明**：包名是巡检方向遗留，当前课题为复合机器人抓取。抓取方向需要**新增**若干 msg/srv；原巡检相关 msg/srv（`DefectInfo` 等）保留定义以避免破坏历史代码，但不再扩展。

目标：
- 避免各包各自定义同义字段导致漂移
- 让 `inspection_gateway` 有稳定的 ROS2 接口可调用/订阅
- 覆盖抓取任务完整 pipeline（NAV / OBSERVE / PERCEIVE / GRASP / PLACE）

## 1. 边界与优先级

优先级（从高到低）：
1. `inspection_gateway/api/models.py`（Pydantic v2，对外 REST/WS API 契约）
2. `inspection_interface`（对内 ROS2 契约）
3. 厂商协议（只允许出现在各 driver 内）

原则：
- `inspection_interface` 应尽量与 `models.py` 中的 Pydantic 模型保持语义一致
- 抓取方向新增的 ROS2 消息与网关侧 Pydantic 模型**字段命名一致**（`object_class` / `grasp_pose` / `media_id` 等）

## 2. 抓取方向新增（见 TODO.md P0）

### 新增 msg
- `GraspPose.msg` — `{geometry_msgs/Pose pose; float64 width_m; float64 score; string frame_id}`
- `GripperStatus.msg` — `{builtin_interfaces/Time stamp; bool is_closed; bool moving; string error_code}`
- `GraspEvent.msg`（或合并进 `SystemState`）— `{string type; string task_id; string phase; string media_id; geometry_msgs/Pose grasp_pose; string message; builtin_interfaces/Time stamp}`

### 新增 srv
- `PerceiveGrasp.srv`
  ```
  string object_class           # 可选，空字符串表示"任意"
  ---
  bool success
  string error
  GraspPose[] candidates        # frame_id 在 GraspPose 内
  string frame_id               # 所有候选共享的坐标系（一般是 camera_color_optical_frame）
  ```
- `StartGrasp.srv`（或扩充原 `StartInspection.srv`）
  ```
  string task_name              # 查 grasp_tasks.yaml
  string object_class           # 可选
  bool dry_run                  # true 只做感知+规划，不真动
  ---
  bool success
  string task_id
  string error
  ```
- `SetDioOutput.srv` — 已有（dio_driver 使用）；`gripper_driver` 对 `/inspection/gripper/open|close` 直接用 `std_srvs/Trigger`

### 待调整 msg
- `SystemState.msg` — `phase` 枚举扩充为抓取阶段；新增 `gripper_status` 字段

### 保留（不再扩展，也不删）
- `AgvStatus.msg` / `ArmStatus.msg` / `GetNavMap.srv` / `MoveToPose.srv` / `MoveToJoints.srv`
- `DefectInfo.msg`（巡检遗留；保留字段不破坏历史消息库）

## 3. 演进规则（必须遵守）

1. 只追加字段，不复用字段号
2. 不在 README/文档里复制字段表（以 `.msg/.srv` 为准）
3. 修改字段语义必须同步更新：
   - `inspection_gateway/api/models.py`（若对外语义变化）
   - `docs/WORKSPACE_OVERVIEW.md`（端到端约定）
   - `docs/IMPLEMENTATION_STATUS.md`（落地缺口）

## 4. 推荐组织方式

当 msg/srv 增长后，建议按域拆分命名：
- AGV/Arm/Gripper：`AgvStatus` / `ArmStatus` / `GripperStatus` / `SetDioOutput`
- 感知：`GraspPose` / `PerceiveGrasp`
- 任务：`StartGrasp` / `PauseInspection` / `GraspEvent` / `SystemState`
- 导航：`GetNavMap`

并保持"对外 REST API 语义优先"的命名（`grasp_pose`、`object_class` 等，而不是 `target_pose`、`class_name`）。

## 5. 文档与 TODO 维护（必须）

- 任何 msg/srv 变更必须同步更新：`docs/WORKSPACE_OVERVIEW.md`、`docs/IMPLEMENTATION_STATUS.md`、仓库根 `TODO.md`
- 破坏性变更必须在提交信息中标注 `!` 或 `BREAKING CHANGE:`
- 抓取方向的新增消息**必须**与 `api/models.py` 的对应 Pydantic 模型同步（字段命名 + 类型）
