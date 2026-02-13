# 项目关系与 AGV 职责梳理

## 1. 项目关系

根据 `inspection-robot/README.md` 与 `inspection-api/README.md`：

- `inspection-api` 是协议契约仓库，只定义 `inspection_gateway` 的外部接口（gRPC/OpenAPI）。
- `inspection-robot` 是 ROS2 执行仓库，包含驱动层、控制层、算法层、协调层。
- `inspection_gateway` 位于 `inspection-robot`，对外承接 `inspection-hmi` 请求，对内调用 ROS2 节点。
- `task_coordinator` 是 `inspection-robot` 的流程编排核心，统一调度 AGV、机械臂、相机与算法节点。

整体调用链可简化为：

`inspection-hmi -> inspection_gateway -> task_coordinator -> agv_driver/arm_controller/...`

## 2. AGV 在系统中的关键职责

`agv_driver` 的职责不是规划，而是“执行 + 状态反馈”：

- 接收 `/inspection/agv/goal_pose`（`map` 坐标系目标位姿）并下发到底盘导航。
- 可选接收 `/inspection/agv/cmd_vel`，用于调试开环运动。
- 持续发布 `/inspection/agv/current_pose`。
- 发布 `/inspection/agv/status`，至少包含：
  - `arrived`：导航到位
  - `stopped`：底盘静止
  - `error_code`：故障码/错误态
  - `battery_percent`：电量
- （规划中）提供导航地图能力，供网关对外实现 `GetNavMap`：
  - service：`/inspection/agv/get_nav_map`（`inspection_interface/srv/GetNavMap`）
- 必要时发布 `/inspection/agv/odom` 和 `map -> base_link` TF。

## 3. 与任务流程的耦合点（必须满足）

`task_coordinator` 主流程里，AGV 相关门控条件是：

- 先发送 AGV 目标位姿。
- 只有当 `arrived=true && stopped=true` 时，才允许下发机械臂动作。

因此 AGV 驱动必须稳定提供“到位+静止”双条件，不能只给其中一个。

## 4. AGV 的最小可运行闭环

1. 建立 TCP 连接并完成控制权检查/抢占（若启用控制权机制）。
2. 确认地图已载入、定位可用（必要时重定位）。
3. 执行导航任务（到站点或到任意坐标）。
4. 轮询或推送获取位姿、导航状态、速度与阻挡/急停状态。
5. 输出 ROS 状态字段，供 `task_coordinator` 判定是否可进入机械臂步骤。
6. 异常时支持取消导航、软急停、清错等恢复动作。
