# inspection_sim/CLAUDE.md

本包提供 **无实物（no-hardware）联调环境**：用 fake 节点替代 `agv_driver / arm_driver / realsense_driver / grasp_perception / gripper_driver` 的外部依赖，保证 `task_coordinator` 与 `inspection_gateway` 可在纯软件环境下跑通抓取 pipeline。

> **方向变更**：原 fake 节点是为巡检设计的（`fake_perception/detect` + `fake_planning/optimize` + `fake_defect`）。抓取方向需要替换为：
> - `fake_grasp_perception`：提供 `perceive_grasp` service（返回固定或可配置的抓取候选）
> - `fake_gripper`：提供 `gripper/open` / `gripper/close` service
> - `fake_realsense`：发布稳定的 RGB/Depth/camera_info + TF（可用内置样例图）
> 原 `fake_pose_detector / fake_path_planner / fake_defect` 的设计已不适用。

## 1. 包职责与边界

负责：
- 提供"接口级仿真"节点（fake drivers），复刻真实包的 Public ROS API
- 提供可重复的时序/延迟/噪声注入（用于实验对比）
- 提供一键启动的仿真 launch

不负责：
- 真实硬件通信（厂商协议只能在 `*_driver` 内）
- 真实感知/规划算法（应落在 `grasp_perception` 和 `arm_controller` 的 Core）
- 对外 REST/WS API（由 `inspection_gateway` 负责）

## 2. Public ROS API（目标接口，必须对齐）

### 2.1 Fake AGV（对齐 `agv_driver`）

namespace：`/inspection/agv`

- 订阅：`goal_pose`、`goal_station`、`cmd_vel`（可选）
- 发布：`status`、`current_pose`、`odom`（可选）、TF（可选）
- 服务：`get_nav_map` (`inspection_interface/srv/GetNavMap`)

### 2.2 Fake Arm（对齐 `arm_driver`）

namespace：`/inspection/arm`

- 订阅：`joint_cmd`（可选）
- 发布：`/joint_states`、`status`

默认关节名（Elfin5）：`elfin_joint1..elfin_joint6`

### 2.3 Fake Arm Controller（对齐 `arm_controller`）

namespace：`/inspection/arm_control`

- 服务：`move_to_pose` / `move_to_joints`（立即返回 success，可选延迟模拟）

### 2.4 Fake RealSense（对齐 `realsense_driver`）【新】

namespace：`/inspection/realsense/d435`

- 发布：`color/image_raw` / `aligned_depth_to_color/image_raw` / `color/camera_info`
  - 可回放内置样例 RGBD 图像
- 发布 TF：`camera_link` / `camera_color_optical_frame`

### 2.5 Fake Grasp Perception（对齐 `grasp_perception`）【新】

namespace：`/inspection/grasp`

- 服务：`perceive_grasp` (`inspection_interface/srv/PerceiveGrasp`)
  - 返回固定的抓取候选（可通过 yaml 参数配置 candidates 数量/位姿/score）
  - 可选注入：`no_target_prob`（概率性返回 empty，用于测试 PERCEIVE 重试逻辑）

### 2.6 Fake Gripper（对齐 `gripper_driver`）【新】

namespace：`/inspection/gripper`

- 服务：`open` / `close` (`std_srvs/srv/Trigger`)
- 发布：`status` (`inspection_interface/msg/GripperStatus`)
  - 调用 `close` 后 300ms 内把 `is_closed` 翻为 true

## 3. 推荐内部架构（Node + Core 模型）

目标：避免所有逻辑堆在 Node 回调里，保持可测试与可替换。

建议目录：

- `inspection_sim/nodes/`
  - `fake_agv_node.py`（已有）
  - `fake_arm_node.py`（已有）
  - `fake_arm_controller_node.py`【新】
  - `fake_realsense_node.py`【新】
  - `fake_grasp_perception_node.py`【新，替代 fake_pose_detector / fake_planning / fake_defect】
  - `fake_gripper_node.py`【新】
- `inspection_sim/core/`
  - `angles.py`：角度归一化工具
  - `png.py`：地图底图 PNG 生成（供 GetNavMap）
  - `rgbd_samples.py`【新】：内置样例 RGBD 生成（棋盘格/示例点云）

约束：
- Node 只做 ROS IO（pub/sub/srv/timer），不写复杂状态机
- 核心算法（联合优化、感知推理）不应落在本包；本包只为其提供"可控输入/可控执行环境"

## 4. 数据流（无硬件闭环，抓取方向）

```mermaid
flowchart LR
  GW["inspection_gateway"] -->|StartGrasp| CO["task_coordinator"]
  CO -->|goal_pose| AGV["fake_agv"]
  AGV -->|status| CO

  CO -->|MoveToJoints (observe/home)| ARMC["fake_arm_controller"]
  CO -->|MoveToPose (PILZ_LIN)| ARMC
  ARMC -->|joint_cmd| ARM["fake_arm"]
  ARM -->|/joint_states + status| CO

  RS["fake_realsense"] -->|RGBD + info| GP["fake_grasp_perception"]
  CO -->|PerceiveGrasp| GP
  GP -->|candidates| CO

  CO -->|gripper/open or gripper/close| GR["fake_gripper"]
  GR -->|status| CO

  CO -->|/inspection/state| GW
```

## 5. 与 bringup 的集成约定

目标：在不改上层代码的情况下切换"真机/仿真"。

建议：
- 新增/维护 `inspection_sim/launch/sim_system.launch.py`
  - 启动 fake nodes + `task_coordinator` + `inspection_gateway`
  - 不启动 `agv_driver / arm_driver / realsense_driver / grasp_perception / gripper_driver`（避免冲突）
- 或者给 `inspection_bringup/system.launch.py` 增加 `use_sim:=true` 分支（保持接口不变）

## 6. 文档与 TODO 维护（必须）

- 本仓库根 `TODO.md` 是单一事实来源：仿真相关未完成项必须写进 `TODO.md`
- 若 `inspection_sim` 的对外接口（topic/service/参数）变化，必须同步更新：
  - 本文件 + `README.md`
  - `docs/ARCHITECTURE.md`（ROS 接口章节）
  - `docs/WORKSPACE_OVERVIEW.md`（端到端口径，如受影响）
  - 仓库根 `TODO.md`
