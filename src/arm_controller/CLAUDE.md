# arm_controller/CLAUDE.md

本文件约束 `arm_controller` 的架构与数据流。当前主线为**巡检系统中的机械臂运动控制**。

## 1. 包职责与边界

负责：

- 使用 MoveIt2 将目标关节角或末端目标位姿规划为机械臂轨迹。
- 对外提供 `move_to_joints` / `move_to_pose` service。
- 将规划结果下发给 `arm_driver`。
- 发布运动状态和执行进度。
- 支撑 `task_coordinator` 的 `ARM_PRESET` 和 `DEPTH_ADJUST` 阶段。

不负责：

- EtherCAT 硬件通信。
- 巡检任务状态机。
- 深度图处理和工作距估计算法。
- 缺陷检测算法。

## 2. Public ROS API

默认命名空间：`/inspection/arm_control`

订阅：

- `cart_goal` (`geometry_msgs/msg/PoseStamped`)：调试入口。
- `joint_goal` (`sensor_msgs/msg/JointState`)：关节目标调试入口。
- `velocity_scaling` (`std_msgs/msg/Float64`)：速度缩放。

发布：

- `motion_status` (`std_msgs/msg/String`)
- `trajectory_progress` (`std_msgs/msg/Float64`)

服务：

- `move_to_pose` (`inspection_interface/srv/MoveToPose`)
- `move_to_joints` (`inspection_interface/srv/MoveToJoints`)

下游：

- `arm_driver_joint_cmd_topic` 默认 `/inspection/arm/joint_cmd`
- `arm_driver_enable_service` 默认 `/inspection/arm/enable`

## 3. 与巡检状态机的关系

- `ARM_PRESET`：`task_coordinator` 调 `move_to_joints`，机械臂到站位预设观测姿态。
- `DEPTH_ADJUST`：`task_coordinator` 根据 RealSense 工作距误差生成新的 TCP 绝对目标位姿，调 `move_to_pose` 做小范围修正。

注意：`move_to_pose` 当前按 planning frame 下的**绝对目标位姿**执行。调用方不能把 `target_pose.position` 当作相对偏移。

## 4. 推荐内部架构

当前实现集中在 `ArmControllerNode` 中，后续建议拆为：

1. `MoveItFacade`
   - 初始化 `MoveGroupInterface`
   - 设置 planning time / velocity scaling / target
   - 管理 planning frame 和 current pose 查询
2. `TrajectoryExecutor`
   - `execute_final_point` / `execute_trajectory`
   - 进度发布与错误处理
3. `DriverBridge`
   - auto-enable
   - 下发 `joint_cmd`
   - topic/service 名称集中管理

## 5. 文档与 TODO 维护

修改 public ROS API、规划语义或执行策略时，必须同步更新：

- `TODO.md`
- `docs/ARCHITECTURE.md`
- `docs/IMPLEMENTATION_STATUS.md`
- `src/task_coordinator/CLAUDE.md`

