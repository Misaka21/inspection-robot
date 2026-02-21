# inspection_gateway — CLAUDE.md

本包是 **Web HMI ↔ ROS2 的桥接层**，替代了原有的 gRPC 方案（`inspection-api` + `inspection-hmi` 已归档）。

```
浏览器 (任意设备) ──REST/WS──→ inspection_gateway (FastAPI, AGX :8080) ──ROS2──→ drivers/planners/coordinator
```

前端源码在独立的 `inspection-site` 仓库中开发，`npm run build` 产物部署到 `frontend/dist/`，由 FastAPI 托管。

---

## 1. 包职责与边界

**负责：**
- REST API（`/api/v1/...`）+ WebSocket（`/ws`）对外接口
- ROS2 topic/service 桥接
- 文件存储：CAD 模型（`CadStore`）、媒体资源（`MediaStore`）
- 前端静态文件托管（`frontend/dist/`）

**不负责：**
- 设备驱动协议（在各 `*_driver` 内）
- 规划/检测算法（在 `path_planner` / `defect_detector` 内，gateway 只编排调用）
- 前端源码（在 `inspection-site` 仓库）

---

## 2. 分层架构

```
inspection_gateway/
├── main.py                 # 进程入口: ROS2 守护线程 + uvicorn 主线程
├── api/
│   ├── app.py              # FastAPI 工厂 (create_app)
│   ├── deps.py             # 依赖注入 (get_bridge / get_state_hub / ...)
│   ├── models.py           # Pydantic v2 数据模型 — API 契约的唯一事实来源
│   ├── routes/
│   │   ├── cad.py          # POST /api/v1/cad/upload
│   │   ├── targets.py      # POST /api/v1/targets              ← 桩代码
│   │   ├── plans.py        # POST/GET /api/v1/plans            ← 桩代码
│   │   ├── tasks.py        # POST /api/v1/tasks + pause/resume/stop/status
│   │   ├── nav.py          # GET /api/v1/nav/map
│   │   ├── media.py        # GET /api/v1/media/{id}
│   │   └── captures.py     # GET /api/v1/tasks/{id}/captures   ← 桩代码
│   └── ws/
│       ├── handler.py      # WebSocket /ws 端点
│       └── protocol.py     # WsEnvelope 消息信封
├── ros/
│   ├── bridge.py           # RosBridge: ROS2 service clients (线程安全)
│   └── state_hub.py        # StateHub: ROS2 → WebSocket 线程安全 pub/sub
├── store/
│   ├── cad_store.py        # SHA256 内容寻址 CAD 文件存储
│   └── media_store.py      # SHA256 内容寻址媒体文件存储
├── domain/
│   ├── converters.py       # ROS msg → Pydantic model 纯转换函数
│   └── runtime.py          # GatewayRuntime (task_id/plan_id/task_name 运行时状态)
└── frontend/
    └── dist/               # inspection-site 构建产物, FastAPI 静态托管
```

**强制约束：**
- `main.py` 只做进程启动、参数解析、线程模型装配，**禁止**实现具体业务
- route handler 保持薄：同步等待 ROS srv，转换结果返回，不做复杂业务逻辑
- 所有 Pydantic 模型集中在 `models.py`，对应原 `inspection-api/proto/inspection_gateway.proto`

---

## 3. REST API 端点 — 实现状态

| HTTP | Path | 实现状态 | 说明 |
|------|------|---------|------|
| POST | `/api/v1/cad/upload` | **已实现** | multipart/form-data, SHA256 内容寻址写入 CadStore |
| POST | `/api/v1/targets` | **桩代码** | 仅返回 `total_targets=len(targets)`，不存储不转发 |
| POST | `/api/v1/plans` | **桩代码** | 返回 `UNAVAILABLE` |
| GET | `/api/v1/plans/{plan_id}` | **桩代码** | 返回 `NOT_FOUND` |
| POST | `/api/v1/tasks` | **已实现** | 调用 `RosBridge.start_inspection()` |
| POST | `/api/v1/tasks/{id}/pause` | **已实现** | 调用 `RosBridge.pause_inspection()` |
| POST | `/api/v1/tasks/{id}/resume` | **已实现** | 调用 `RosBridge.resume_inspection()` |
| POST | `/api/v1/tasks/{id}/stop` | **已实现** | 调用 `RosBridge.stop_inspection()` |
| GET | `/api/v1/tasks/{id}/status` | **已实现** | 调用 `RosBridge.get_status()`, converters 转换 |
| GET | `/api/v1/nav/map` | **已实现** | 调用 `RosBridge.get_nav_map()`, 图片写入 MediaStore |
| GET | `/api/v1/media/{media_id}` | **已实现** | StreamingResponse 256KB 分块 |
| GET | `/api/v1/tasks/{id}/captures` | **桩代码** | 返回 `UNAVAILABLE` |
| WS | `/ws` | **部分实现** | 只推送 `system_state`，不推送 `inspection_event` |

OpenAPI 文档自动生成: `http://localhost:8080/docs`

### 桩代码待实现清单

1. **`POST /targets`** — 需要：接收 `SetTargetsRequest`，将 targets 持久化（内存或文件），关联到 `model_id`
2. **`POST /plans`** — 需要：将 targets 转发给 ROS2 `path_planner`（或新增 ROS2 srv），返回 `InspectionPath` + `plan_id`
3. **`GET /plans/{plan_id}`** — 需要：从缓存中读取已规划的路径，用于断点恢复/重连
4. **`GET /tasks/{id}/captures`** — 需要：订阅 `InspectionEvent`（`CAPTURED`/`DEFECT_FOUND`），持久化到 store 中，按 task_id/point_id 查询
5. **WebSocket `inspection_event`** — 需要：订阅 ROS2 的检测事件 topic，转为 `inspection_event` 类型推送给前端

---

## 4. WebSocket 协议

单一端点 `/ws`，通过 `type` 字段区分消息类型：

```json
{
  "type": "system_state" | "inspection_event" | "error" | "ping" | "pong",
  "seq": 42,
  "timestamp": "2026-02-21T10:30:00.123Z",
  "payload": { ... }
}
```

**当前实现：**
- `system_state` — 推送 `TaskStatus`（含 AGV/Arm/phase/progress/interlock），来自 `StateHub` 订阅 ROS2 `SystemState` topic
- `ping` / `pong` — 客户端发 ping，服务端回 pong

**待实现：**
- `inspection_event` — 需要订阅 ROS2 检测事件 topic，转为 `InspectionEvent` 推送（CAPTURED/DEFECT_FOUND/WARN/ERROR）

---

## 5. ROS2 桥接

### RosBridge 服务客户端

| ROS2 名称 | 接口类型 | 对应 HTTP |
|-----------|---------|-----------|
| `/inspection/start` | `StartInspection.srv` | `POST /tasks` |
| `/inspection/pause` | `PauseInspection.srv` | `POST /tasks/{id}/pause` |
| `/inspection/resume` | `ResumeInspection.srv` | `POST /tasks/{id}/resume` |
| `/inspection/stop` | `StopInspection.srv` | `POST /tasks/{id}/stop` |
| `/inspection/get_status` | `GetInspectionStatus.srv` | `GET /tasks/{id}/status` |
| `/inspection/agv/get_nav_map` | `GetNavMap.srv` | `GET /nav/map` |

### StateHub 订阅

| ROS2 名称 | 消息类型 | 对应 WS |
|-----------|---------|---------|
| `/inspection/state` | `SystemState.msg` | `system_state` |

### 缺失的 ROS2 接口（桩代码需要的）

- 规划服务（`SetTargets` + `PlanInspection`）— 需要 `path_planner` 或 `task_coordinator` 提供 ROS2 srv
- 检测事件 topic — 需要 `task_coordinator` 或 `defect_detector` 发布 `InspectionEvent` 消息

---

## 6. 并发模型

```
┌──────────────────┐     ┌──────────────────────────────────┐
│ ROS2 Executor    │     │ uvicorn (asyncio 主线程)          │
│ (守护线程)        │     │                                  │
│                  │     │  HTTP handlers (sync, threadpool) │
│ SystemState.msg ─┼──→ StateHub ──→ WebSocket handler      │
│  callback        │     │                                  │
└──────────────────┘     └──────────────────────────────────┘
```

- ROS2 executor 在守护线程中运行，处理订阅回调和 service client 完成回调
- uvicorn 异步事件循环在主线程，处理 HTTP 和 WebSocket
- `StateHub` 通过线程安全的 `_LatestQueue(maxsize=1)` 桥接两个线程域
- `RosBridge._call()` 使用 `threading.Lock` 序列化 ROS2 service 调用

---

## 7. 数据模型溯源

`api/models.py` 中的 Pydantic v2 模型直接对应原 `inspection-api/proto/inspection_gateway.proto` 的消息定义：

| Proto 消息 | Pydantic 模型 | 说明 |
|-----------|--------------|------|
| `Pose2D` | `Pose2D` | `{x, y, yaw, frame_id}` |
| `InspectionTarget` | `InspectionTarget` | `{point_id, group_id, surface, view}` |
| `InspectionPoint` | `InspectionPoint` | `{agv_pose, arm_joint_goal[6], tcp_pose_goal, camera_pose}` |
| `InspectionPath` | `InspectionPath` | `{waypoints[], estimated_distance_m, estimated_duration_s}` |
| `TaskStatus` | `TaskStatus` | 含 `AgvStatus` + `ArmStatus` + interlock + progress |
| `InspectionEvent` | `InspectionEvent` | `{type, image, defects[], camera_pose}` |
| `NavMapInfo` | `NavMapInfo` | `{resolution, origin, image}` |

前端 `inspection-site/src/api/types.ts` 是这些模型的 TypeScript 镜像。**三层必须保持一致。**

---

## 8. 依赖

- `rclpy`, `inspection_interface`
- `fastapi>=0.100.0`, `uvicorn[standard]>=0.23.0`, `pydantic>=2.0.0`, `python-multipart`

---

## 9. 开发与部署

```bash
# 开发模式 (两个终端)
# Terminal 1: FastAPI + ROS2
source /opt/ros/humble/setup.bash && source install/setup.bash
python -m inspection_gateway.main --port 8080

# Terminal 2: 前端开发服务器 (在 inspection-site 目录)
cd inspection-site && npm run dev  # :5173, 代理 /api 和 /ws 到 :8080

# 生产部署
cd inspection-site && npm run build  # 输出到 gateway/frontend/dist/
python -m inspection_gateway.main --port 8080  # 一个端口搞定
```

---

## 10. 与原架构的对应关系

```
原架构:  inspection-api (proto) ← inspection-hmi (Qt C++ gRPC) → inspection_gateway (gRPC server)
现架构:  inspection-site (React) ← REST/WS → inspection_gateway (FastAPI) → ROS2
```

- `inspection-api` → `api/models.py` (Pydantic) + OpenAPI 自动文档
- `inspection-hmi/GatewayClient.cpp` → `inspection-site/src/api/` (fetch + WebSocket)
- `inspection-hmi/OperatorWindow` → `inspection-site/src/pages/OperatorPage.tsx`
- `inspection-hmi/MainWindow` → `inspection-site/src/pages/EngineerPage.tsx` (待实现)
