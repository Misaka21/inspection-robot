# 实现状态与代码规划（对齐 REST/WS 网关）

本文档把 `inspection_gateway` 的 REST/WS 对外能力，映射到 `inspection-robot` 的包结构/ROS2 接口，并标注**当前代码是否已存在**，避免"文档写了但代码没落地"。

> 历史说明：本项目最初采用 gRPC 架构（`inspection-api` proto + `inspection-hmi` Qt），已于 2025 年迁移至 FastAPI REST/WS 方案。`inspection-api` 和 `inspection-hmi` 仓库已归档，当前前端为 `inspection-site`（React）。

## 1. 当前代码现状（快速结论）

- `inspection_gateway`（FastAPI REST/WS server）：**已实现最小可用闭环（P0 子集）**（`src/inspection_gateway/`）
  - 已有：FastAPI server 启动、ROS2 bridge、CAD/媒体落盘、`Start/Pause/Resume/Stop/GetTaskStatus/SystemState(WS)/GetNavMap/DownloadMedia`
  - 已有：`POST /targets`（存储 targets 到内存）、`POST /plans`（网关侧弧形路径生成）、`GET /plans/{plan_id}`（缓存查询）
  - 缺失：`captures` 的落盘与查询、`inspection_event` WebSocket 推送
- 导航地图 `GetNavMap`：
  - ROS2 `inspection_interface/srv/GetNavMap`：**已定义**
  - `agv_driver` `get_nav_map` service server：**缺失**（仿真 `fake_agv` 已提供）
  - AGV 厂商地图 API 封装：**缺失**
- 规划 `PlanInspection`：
  - `path_planner` ROS2 节点目前只有 `Trigger` 骨架，**不产出 `InspectionPath` 语义**
  - `inspection_gateway/routes/plans.py` 提供了网关侧弧形路径生成（临时方案，未调用 ROS2 `path_planner`）
- 任务编排 `task_coordinator`：状态机骨架已实现，**step 3 机械臂目标下发已修复**（通过 `path_detail` 获取关节角并下发），**超时机制已启用**；仍缺失缺陷结果订阅
- 取图链路：`hikvision_driver` 已有 `trigger_capture` + `image_raw`，但**没有媒体落盘/媒体 id/事件流**的实现

## 2. REST/WS 能力到机器人端模块映射（V1）

说明：
- "当前状态"仅描述仓库内是否已有可用模块，不代表算法正确性。
- 下面的"建议 ROS2 接口"是对内稳定契约，供 `inspection_gateway` 调用。

| REST/WS 端点 | 机器人端责任模块 | ROS2 接口（对内） | 当前状态 |
|---|---|---|---|
| `POST /cad/upload` | `inspection_gateway` + `CadStore` | （网关内处理即可） | **已实现**（落盘 + SHA256 model_id） |
| `POST /targets` | `inspection_gateway` + `GatewayRuntime` | （网关内存存储） | **已实现**（存储到 `runtime.targets_by_model`） |
| `POST /plans` | `inspection_gateway` -> `path_planner`（目标） | `inspection_interface/srv/PlanInspection` | **网关侧已实现**（弧形路径生成），未调用 ROS2 path_planner |
| `GET /plans/{plan_id}` | `inspection_gateway` + 内存缓存 | （网关内处理即可） | **已实现**（从 `runtime.plans` 查询） |
| `POST /tasks` | `inspection_gateway` -> `task_coordinator` | `/inspection/start` (`StartInspection.srv`) | **已实现**（网关 → ROS srv） |
| `POST /tasks/{id}/pause` | `inspection_gateway` -> `task_coordinator` | `/inspection/pause` (`PauseInspection.srv`) | **已实现** |
| `POST /tasks/{id}/resume` | `inspection_gateway` -> `task_coordinator` | `/inspection/resume` (`ResumeInspection.srv`) | **已实现** |
| `POST /tasks/{id}/stop` | `inspection_gateway` -> `task_coordinator` | `/inspection/stop` (`StopInspection.srv`) | **已实现** |
| `GET /tasks/{id}/status` | `inspection_gateway` -> `task_coordinator` | `/inspection/get_status` (`GetInspectionStatus.srv`) | **已实现**（ROS SystemState → REST TaskStatus） |
| `WS system_state` | `inspection_gateway` 订阅 ROS2 状态并推送 | `/inspection/state` (`SystemState.msg`) | **已实现**（StateHub + WebSocket 推送） |
| `WS inspection_event` | `inspection_gateway` 订阅事件并推送 | `/inspection/events`（待新增 msg） | **未实现** |
| `GET /nav/map` | `inspection_gateway` 缓存 + `agv_driver` 提供原始能力 | `/inspection/agv/get_nav_map` (`GetNavMap.srv`) | **网关已实现**（ROS server 真机端缺失，仿真端已有） |
| `GET /tasks/{id}/captures` | `inspection_gateway` + `MediaStore` | （网关内处理即可） | **桩代码**（返回 UNAVAILABLE） |
| `GET /media/{media_id}` | `inspection_gateway` + `MediaStore` | （网关内处理即可） | **已实现**（按 media_id 分块下载） |

## 3. 代码层级结构

### 3.1 inspection_gateway（Python，FastAPI）

当前状态：已建立完整分层架构。

```
inspection_gateway/
├── main.py            # 进程入口: ROS2 守护线程 + uvicorn 主线程
├── api/
│   ├── app.py         # FastAPI 工厂
│   ├── deps.py        # 依赖注入
│   ├── models.py      # Pydantic v2 数据模型 — API 契约的唯一事实来源
│   ├── routes/        # REST 路由（薄层）
│   └── ws/            # WebSocket 端点
├── ros/
│   ├── bridge.py      # RosBridge: ROS2 service clients (线程安全)
│   └── state_hub.py   # StateHub: ROS2 → WebSocket 线程安全桥接
├── store/             # 文件存储（CAD/媒体）
└── domain/
    ├── converters.py  # ROS msg → Pydantic model 纯转换函数
    └── runtime.py     # 运行时状态（plans/targets/task_id）
```

### 3.2 ROS2 侧（inspection-robot）

现有包的边界：

- `agv_driver`: 封装厂商 TCP API + 发布状态 + 提供 `get_nav_map`（待实现）
- `arm_driver`: EtherCAT 驱动 + 发布状态
- `arm_controller`: MoveIt2 执行层（TCP 目标姿态/MoveJ 轨迹）
- `pose_detector`: 工件位姿估计（骨架，算法待实现）
- `path_planner`: 输入 targets + constraints，输出 waypoints（骨架，算法待实现）
- `defect_detector`: 输入图像，输出缺陷结构化结果（骨架，算法待实现）
- `task_coordinator`: 任务状态机与联锁门控
- `inspection_interface`: 对内 msg/srv

## 4. 数据流（对齐"导航 + 机械臂 + 结果回显"）

### 4.1 控制面（Engineer/Operator）

```
浏览器 (inspection-site)
  │
  ├─ POST /cad/upload, POST /targets, POST /plans
  │                         │
  │            inspection_gateway (FastAPI)
  │             ├─ 网关侧弧形路径生成（临时）
  │             └─ 未来: 调用 path_planner ROS2 srv
  │
  ├─ POST /tasks (StartInspection)
  │                         │
  │            inspection_gateway → /inspection/start → task_coordinator
  │
  └─ WS /ws (system_state 实时推送)
                            │
               inspection_gateway ← /inspection/state ← task_coordinator
```

### 4.2 执行面

```
for waypoint in path:
    task_coordinator → /inspection/agv/goal_pose → agv_driver
    wait agv arrived && stopped
    task_coordinator → /inspection/arm_control/cart_goal → arm_controller   [⚠ step 3 空操作]
    task_coordinator → trigger hikvision_driver
    hikvision_driver → image_raw → defect_detector
    task_coordinator → trigger detect_defect                               [⚠ 结果未收集]
```

## 5. 下一步落地顺序（建议）

1. **`agv_driver` 实现 `get_nav_map`** — 封装厂商 TCP API（真机端缺失，仿真端已有）
2. **取图链路闭环** — 引入 `capture_manager`（或等效模块），抓拍落盘生成 `media_id`
3. **`path_planner` 升级** — 从 Trigger 骨架升级为输入 targets + constraints 输出 InspectionPath 的 ROS2 srv
4. **`POST /plans` 对接 ROS2** — 网关调用 `path_planner` 替代当前的网关侧弧形生成
5. **缺陷结果订阅** — `task_coordinator` 订阅 `defect_detector` 结果 topic，不再丢失缺陷详情
6. **结构化事件流** — 新增 `/inspection/events` topic，网关映射到 WebSocket `inspection_event`
