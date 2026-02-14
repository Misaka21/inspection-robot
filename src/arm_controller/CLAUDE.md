# arm_controller/CLAUDE.md

本文件约束 `arm_controller` 的架构与数据流，目标是：**MoveIt2 规划/执行逻辑与 ROS IO 解耦**，避免在回调里写长流程导致线程阻塞。

## 1. 包职责与边界

负责：
- MoveIt2 规划 `PoseGoal -> JointTrajectory`
- 按策略把轨迹下发给 `arm_driver`（stream 或最终点）
- 对外提供 `move_to_pose` service（供 `task_coordinator`/网关调用）

不负责：
- EtherCAT 硬件通信（`arm_driver`）
- 任务编排（`task_coordinator`）

## 2. Public ROS API（稳定接口）

默认命名空间：`/inspection/arm_control`

订阅：
- `cart_goal` (`geometry_msgs/msg/PoseStamped`)：火并式执行（plan+execute）
- `joint_goal` (`sensor_msgs/msg/JointState`)：直接透传关节命令（显示/调试）
- `velocity_scaling` (`std_msgs/msg/Float64`)：速度缩放

发布：
- `motion_status` (`std_msgs/msg/String`)：planning/executing/done/failed
- `trajectory_progress` (`std_msgs/msg/Float64`)：0~1

服务：
- `move_to_pose` (`inspection_interface/srv/MoveToPose`)

下游依赖：
- `arm_driver_joint_cmd_topic` 默认 `/inspection/arm/joint_cmd`
- 可选：`arm_driver_enable_service` 默认 `/inspection/arm/enable`

## 3. 推荐内部架构（保持 Node 轻量）

当前实现把逻辑集中在 `ArmControllerNode` 类里，后续扩展建议拆分为 3 个类：

1. `MoveItFacade`
   - 初始化 `MoveGroupInterface`
   - 设置 planning time / scaling / target
2. `TrajectoryExecutor`
   - `execute_final_point` / `execute_trajectory(stream)` 的时序与 sleep
   - 进度发布与错误收敛
3. `DriverBridge`（可选）
   - auto-enable 调用
   - topic/service 名称集中管理，避免散落字符串常量

约束：
- ROS 回调里不要长期阻塞（MoveIt plan + stream 很慢）；建议把执行放到 worker 线程或独立 executor（后续重构时做）。

## 4. 数据流

```mermaid
flowchart LR
  Goal["cart_goal / move_to_pose"] --> Planner["MoveItFacade (plan)"]
  Planner --> Exec["TrajectoryExecutor"]
  Exec -->|publish JointState| Driver["arm_driver"]
  Driver -->|/joint_states| MoveIt["MoveIt state monitor"]
```

## 5. 与 inspection-api 对齐时的落点（后续）

`path_planner` 未来可能产出 `tcp_pose_goal` 或 `arm_joint_goal`：
- `arm_joint_goal` 直接走 `joint_cmd`
- `tcp_pose_goal` 走 `move_to_pose`

## 6. 文档与 TODO 维护（必须）

- 修改 public ROS API（topic/service/参数）时，必须同步更新：本文件、相关 launch/config、`docs/ARCHITECTURE.md`、仓库根 `TODO.md`
- 新增功能但未实现完：必须把未完成项写入 `TODO.md`（带清晰落点与验收标准）
- 完成 TODO：必须勾选并在提交信息/PR 描述里说明验证方式（真机/仿真/回放）
