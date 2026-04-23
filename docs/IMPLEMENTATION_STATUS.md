# 实现状态与代码规划（抓取场景）

本文档把 `inspection_gateway` 的 REST/WS 对外能力，映射到 `inspection-robot` 的包结构 / ROS2 接口，并标注**当前代码是否已存在**，避免"文档写了但代码没落地"。

> **方向变更（2026-04-23）**：从巡检改为抓取。`task_coordinator` 原"8 点位循环"状态机、`defect_detector` 调用、`hikvision_driver` 作为主相机 —— 这些内容在抓取 pipeline 里**不再是主路径**。原 REST/WS 端点中的"巡检任务"语义需要调整为"抓取任务"语义。
>
> 历史演进：本项目最初采用 gRPC 架构（`inspection-api` proto + `inspection-hmi` Qt），已于 2025 年迁移至 FastAPI REST/WS 方案。`inspection-api` 和 `inspection-hmi` 仓库已归档，前端当前为 `inspection-site`（React）。

## 1. 当前代码现状（快速结论）

### 1.1 基础设施（与课题方向无关，保持可用）
- `inspection_gateway`（FastAPI REST/WS server）：**基础框架已实现**
  - 已有：FastAPI server 启动、ROS2 bridge、CAD/媒体落盘、`Start/Pause/Resume/Stop/GetTaskStatus/SystemState(WS)/GetNavMap/DownloadMedia`
  - 语义待对齐：当前是"巡检任务"语义，需改为"抓取任务"
- `inspection_interface`：ROS2 msg/srv 基础框架已有；**需新增**`PerceiveGrasp.srv`、`StartGrasp.srv`、`GraspPose.msg`、`GripperStatus.msg`
- `agv_driver`：**可用**，真机端 `get_nav_map` 缺失（仿真已有）
- `arm_driver`：**可用**（EtherCAT）
- `arm_controller`：**基本可用**，需启用 Pilz LIN plugin 做直线插补
- `realsense_driver`：**可用**（系统包适配）
- `dio_driver`：**可用**（研华 AGX TCA9539 GPIO 扩展）
- `elfin_description`：URDF **可用**

### 1.2 抓取方向新增/改造
- `task_coordinator`：**需要重写状态机**（从"8 点位循环"→"抓取 pipeline"）。原状态机的配置驱动/YAML/超时机制架构可复用，但具体阶段名和业务逻辑要换
- `gripper_driver`：**缺失**，需新建（Python 包即可）
- `grasp_perception`：**缺失**，需新建（YOLOv8 + 深度路径 A）
- `inspection_bringup` 的标定 yaml（mount_calib / handeye_calib / gripper_offset）：**缺失**
- `inspection_sim`：Fake drivers 需新增 `fake_grasp_perception`、`fake_gripper`

### 1.3 已不再是主路径（保留但标记）
- `defect_detector`：巡检专用，抓取场景不调用。保留源码，但从 `system.launch.py` 移除
- `hikvision_driver`：巡检专用工业相机，抓取主用 RealSense。保留驱动但 `system.launch.py` 默认不启用

## 2. REST/WS 能力到机器人端模块映射（抓取 V1）

说明：
- "当前状态"仅描述仓库内是否已有可用模块，不代表算法正确性
- 下面的"建议 ROS2 接口"是对内稳定契约，供 `inspection_gateway` 调用

| REST/WS 端点 | 机器人端责任模块 | ROS2 接口（对内） | 当前状态 | 备注 |
|---|---|---|---|---|
| `POST /cad/upload` | `inspection_gateway` + `CadStore` | (网关内) | **已实现** | 抓取 V1 不强依赖；6D 位姿升级路径会用 |
| `POST /targets` | `inspection_gateway` + `GatewayRuntime` | (网关内) | **已实现** | 抓取 V1 基本不用；保留字段 |
| `POST /tasks` | `inspection_gateway` → `task_coordinator` | `/inspection/start` (`StartInspection.srv`) | **部分已实现** | **需改为 `StartGrasp.srv`**，支持 `object_class`/`task_name` |
| `POST /tasks/{id}/pause` | `inspection_gateway` → `task_coordinator` | `/inspection/pause` | **已实现** | 语义不变 |
| `POST /tasks/{id}/resume` | `inspection_gateway` → `task_coordinator` | `/inspection/resume` | **已实现** | 语义不变 |
| `POST /tasks/{id}/stop` | `inspection_gateway` → `task_coordinator` | `/inspection/stop` | **已实现** | 语义不变 |
| `GET /tasks/{id}/status` | `inspection_gateway` → `task_coordinator` | `/inspection/get_status` | **已实现** | **phase 枚举需改为抓取阶段** |
| `WS system_state` | `inspection_gateway` 订阅 ROS2 并推送 | `/inspection/state` (`SystemState.msg`) | **已实现** | **SystemState 需扩充抓取阶段字段** |
| `WS grasp_event`（新） | `inspection_gateway` 订阅事件并推送 | `/inspection/events`（待新增 msg） | **未实现** | |
| `GET /nav/map` | `inspection_gateway` 缓存 + `agv_driver` | `/inspection/agv/get_nav_map` | **网关已实现**；ROS server 真机端缺失，仿真已有 | |
| `GET /tasks/{id}/captures` | `inspection_gateway` + `MediaStore` | (网关内) | **桩代码**，返回 UNAVAILABLE | 抓取场景也需要（观察图/抓取瞬间图） |
| `GET /media/{media_id}` | `inspection_gateway` + `MediaStore` | (网关内) | **已实现** | |

## 3. 代码层级结构

### 3.1 inspection_gateway（Python，FastAPI）

当前分层架构完整；抓取方向主要是**语义调整**：

```
inspection_gateway/
├── main.py            # 进程入口: ROS2 守护线程 + uvicorn 主线程
├── api/
│   ├── app.py         # FastAPI 工厂
│   ├── deps.py        # 依赖注入
│   ├── models.py      # Pydantic v2 数据模型 — API 契约的唯一事实来源
│   │                  # ⚠ 需要调整：StartInspection→StartGrasp、phase 枚举、事件类型
│   ├── routes/        # REST 路由（薄层）
│   └── ws/            # WebSocket 端点
├── ros/
│   ├── bridge.py      # RosBridge: ROS2 service clients (线程安全)
│   └── state_hub.py   # StateHub: ROS2 → WebSocket 线程安全桥接
├── store/             # 文件存储（CAD/媒体）
└── domain/
    ├── converters.py  # ROS msg → Pydantic model 纯转换函数
    └── runtime.py     # 运行时状态（targets/task_id）
```

### 3.2 ROS2 侧（inspection-robot）

抓取场景下的包边界：

- `agv_driver`：封装厂商 TCP API + 发布状态 + 提供 `get_nav_map`（真机端待实现）
- `arm_driver`：EtherCAT 驱动 + 发布状态
- `arm_controller`：MoveIt2 执行层（OMPL + **Pilz LIN**，后者用于 PRE_GRASP→GRASP 直线段）
- `realsense_driver`：RGBD + camera_info + TF
- `dio_driver`：板载 DIO 通用层
- `gripper_driver` **[新]**：`open`/`close` 语义，底层调 dio
- `grasp_perception` **[新]**：`perceive_grasp` service，RGBD 输入，抓取候选输出
- `task_coordinator`：抓取 pipeline 状态机（重写）
- `inspection_interface`：对内 msg/srv（新增抓取相关）
- `inspection_sim`：Fake drivers（需补抓取场景）

## 4. 数据流（抓取单次任务）

### 4.1 控制面

```
浏览器 (inspection-site)
  │
  ├─ POST /tasks {task_name, object_class, ...}
  │                         │
  │            inspection_gateway → /inspection/start (StartGrasp.srv) → task_coordinator
  │
  └─ WS /ws
       ├─ system_state ← /inspection/state ← task_coordinator
       └─ grasp_event  ← /inspection/events ← task_coordinator  [待实现]
```

### 4.2 执行面

```
task_coordinator 状态机：

NAV_TO_PICK:     task_coordinator → /inspection/agv/goal_pose → agv_driver
                 wait agv_status.arrived && stopped

ARM_TO_OBSERVE:  task_coordinator → /inspection/arm_control/move_to_joints → arm_controller
                 wait arm_status.arrived

CAPTURE_RGBD:    realsense_driver → task_coordinator (缓存 color/aligned_depth/camera_info)

PERCEIVE:        task_coordinator → /inspection/grasp/perceive_grasp → grasp_perception
                                         ↑
                                   订阅 realsense color/depth/info
                 返回：candidates: GraspPose[]  (frame_id = camera_color_optical_frame)

TRANSFORM_IK:    task_coordinator: tf2 lookup camera→arm_base
                 遍历 candidates: /inspection/arm_control/move_to_pose (plan_only=true) 可达性筛选

PRE_GRASP:       task_coordinator → /inspection/arm_control/move_to_pose (planner=PILZ_LIN)

GRASP:           同上（直线下压到抓取点）

CLOSE_GRIPPER:   task_coordinator → /inspection/gripper/close → gripper_driver
                                                                 ↓
                                                             /inspection/dio/set_output → dio_driver → 电磁阀

LIFT:            task_coordinator → /inspection/arm_control/move_to_pose (PILZ_LIN)

NAV_TO_PLACE / PLACE / OPEN_GRIPPER / HOME: 同模式
```

## 5. 下一步落地顺序（建议）

按 P0 难度递增排列：

1. **标定与 TF**（半天）
   - `base_link → arm_base` 静态 TF（写 yaml）
   - `tool0 → gripper_tip` 静态 TF（量一次写死）
   - `tool0 → camera_link` 手眼标定（`easy_handeye2` + ChArUco，半天）
2. **`gripper_driver`**（半天）
   - Python 薄包，封装 dio/set_output 为 open/close 语义
3. **`inspection_interface` 消息扩充**（1-2 小时）
   - `PerceiveGrasp.srv` / `StartGrasp.srv` / `GraspPose.msg` / `GripperStatus.msg`
4. **`arm_controller` 启用 Pilz LIN**（1 小时）
   - MoveIt plugin 配置 + `move_to_pose.srv` 加 `planner_id` 字段
5. **`grasp_perception` v0（YOLOv8 + 深度 + 固定姿态查表）**（2-3 天）
   - 数据集标注 + 训练 + 节点实现
6. **`task_coordinator` 状态机重写**（2-3 天）
   - 新 pipeline + YAML 配置 + RECOVERY
7. **`inspection_gateway` 语义调整**（1 天）
   - models.py 新增 `StartGrasp`/`GraspEvent`/抓取阶段枚举
   - routes/tasks.py 对应改动
8. **`inspection_bringup/system.launch.py` 整合**（半天）
9. **端到端 demo 走通**（1 天联调）
10. **（可选加分）FoundationPose 6D 位姿路径**（3-5 天）

总预估：2-3 周能跑通 V1 demo；加 6D 位姿再加 1 周。
