# inspection_gateway — CLAUDE.md

本包是 **Web HMI ↔ ROS2 的桥接层**，替代了原有的 gRPC 方案（`inspection-api` + `inspection-hmi` 已归档）。

> **方向说明**：当前课题为复合机器人抓取；包名和 `/api/v1/*` 下某些端点是巡检方向遗留。
> 需要按**抓取任务**语义调整的部分集中在：
> - `api/models.py` 的请求/响应字段（`StartGrasp` / `GraspEvent` / `TaskStatus.phase` 枚举）
> - `routes/tasks.py` 的请求体解析
> - `StateHub` / 新增 `EventHub` 的消息订阅
> 详情见本文件 §3 的"抓取方向待调整"清单。

```
浏览器 (任意设备) ──REST/WS──→ inspection_gateway (FastAPI, AGX :8080) ──ROS2──→ drivers/controller/coordinator/...
```

前端源码在独立的 `inspection-site` 仓库中开发，`npm run build` 产物部署到 `frontend/dist/`，由 FastAPI 托管。

---

## 1. 包职责与边界

**负责：**
- REST API（`/api/v1/...`）+ WebSocket（`/ws`）对外接口
- ROS2 topic/service 桥接
- 文件存储：CAD 模型（`CadStore`，6D 位姿升级路径使用）、媒体资源（`MediaStore`）
- 前端静态文件托管（`frontend/dist/`）

**不负责：**
- 设备驱动协议（在各 `*_driver` 内）
- 感知算法（在 `grasp_perception` 内，gateway 只编排调用）
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
│   │                       # ⚠ 抓取方向需扩：StartGrasp / GraspEvent / phase 枚举
│   ├── routes/
│   │   ├── cad.py          # POST /api/v1/cad/upload（6D 路径使用，V1 可留不拆）
│   │   ├── targets.py      # POST /api/v1/targets（巡检遗留，抓取 V1 不用）
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
│   └── runtime.py          # GatewayRuntime (task_id/task_name 运行时状态)
└── frontend/
    └── dist/               # inspection-site 构建产物, FastAPI 静态托管
```

**强制约束：**
- `main.py` 只做进程启动、参数解析、线程模型装配，**禁止**实现具体业务
- route handler 保持薄：同步等待 ROS srv，转换结果返回，不做复杂业务逻辑
- 所有 Pydantic 模型集中在 `models.py`，抓取方向的新增字段也写在这里

---

## 3. REST API 端点 — 实现状态

| HTTP | Path | 实现状态 | 抓取方向待调整 |
|------|------|---------|---------------|
| POST | `/api/v1/cad/upload` | **已实现** | 抓取 V1 可不使用；6D 位姿路径保留 |
| POST | `/api/v1/targets` | **已实现** | 抓取场景不需要 CAD 表面选点，**可标记为 deprecated** |
| POST | `/api/v1/tasks` | **已实现**（调 StartInspection） | **需改为 `StartGrasp`**，请求体含 `object_class` / `task_name` |
| POST | `/api/v1/tasks/{id}/pause` | **已实现** | 不变 |
| POST | `/api/v1/tasks/{id}/resume` | **已实现** | 不变 |
| POST | `/api/v1/tasks/{id}/stop` | **已实现** | 不变 |
| GET | `/api/v1/tasks/{id}/status` | **已实现** | `phase` 枚举需扩（抓取阶段） |
| GET | `/api/v1/nav/map` | **已实现** | 不变 |
| GET | `/api/v1/media/{media_id}` | **已实现** | 不变 |
| GET | `/api/v1/tasks/{id}/captures` | **桩代码** | 抓取场景需要（观察图/抓取瞬间图） |
| WS | `/ws` | **部分实现** | 需新增 `grasp_event` 类型（替代 `inspection_event` 语义） |

OpenAPI 文档自动生成: `http://localhost:8080/docs`

### 抓取方向待实现清单

1. **`StartGrasp` 请求模型**（`api/models.py` 新增）
   ```python
   class StartGraspRequest(BaseModel):
       task_name: str
       object_class: str | None = None
       dry_run: bool = False
       override_pick: PickConfig | None = None
       override_place: PlaceConfig | None = None
   ```
2. **`TaskStatus.phase` 枚举扩充**：`IDLE / NAV_TO_PICK / ARM_TO_OBSERVE / CAPTURE_RGBD / PERCEIVE / TRANSFORM_IK / PRE_GRASP / GRASP / CLOSE_GRIPPER / LIFT / NAV_TO_PLACE / PLACE / OPEN_GRIPPER / HOME / RECOVERY / COMPLETED / FAILED / STOPPED`
3. **`GraspEvent` 模型**（替代 `InspectionEvent`）：
   ```python
   class GraspEvent(BaseModel):
       type: Literal["OBSERVED","PERCEIVED","GRASPED","PLACED","FAILED"]
       task_id: str
       phase: str
       media_id: str | None = None
       grasp_pose: Pose | None = None  # arm_base frame
       message: str | None = None
       timestamp: datetime
   ```
4. **WebSocket `grasp_event` 订阅**：新增 `EventHub`，订阅 ROS2 `/inspection/events` topic，转为 `grasp_event` 推送
5. **`GET /tasks/{id}/captures`**：订阅 `GraspEvent`，按 task_id 归档 media_id 列表
6. **过渡策略**：`StartInspection.srv` 可保留，由 `task_coordinator` 同时处理新旧请求；或一刀切改名为 `StartGrasp.srv`（推荐，更清晰）

---

## 4. WebSocket 协议

单一端点 `/ws`，通过 `type` 字段区分消息类型：

```json
{
  "type": "system_state" | "grasp_event" | "error" | "ping" | "pong",
  "seq": 42,
  "timestamp": "2026-04-23T10:30:00.123Z",
  "payload": { ... }
}
```

**当前实现：**
- `system_state` — 推送 `TaskStatus`（含 AGV/Arm/phase/progress），来自 `StateHub` 订阅 ROS2 `SystemState` topic
- `ping` / `pong` — 客户端发 ping，服务端回 pong

**抓取方向待实现：**
- `grasp_event` — 订阅 ROS2 `/inspection/events` topic，转为 `GraspEvent` 推送
- `system_state` 的 payload 扩展 `gripper_status`

---

## 5. ROS2 桥接

### RosBridge 服务客户端

| ROS2 名称 | 接口类型 | 对应 HTTP |
|-----------|---------|-----------|
| `/inspection/start` | `StartInspection.srv` → **改为 `StartGrasp.srv`** | `POST /tasks` |
| `/inspection/pause` | `PauseInspection.srv` | `POST /tasks/{id}/pause` |
| `/inspection/resume` | `ResumeInspection.srv` | `POST /tasks/{id}/resume` |
| `/inspection/stop` | `StopInspection.srv` | `POST /tasks/{id}/stop` |
| `/inspection/get_status` | `GetInspectionStatus.srv` | `GET /tasks/{id}/status` |
| `/inspection/agv/get_nav_map` | `GetNavMap.srv` | `GET /nav/map` |

### StateHub 订阅

| ROS2 名称 | 消息类型 | 对应 WS |
|-----------|---------|---------|
| `/inspection/state` | `SystemState.msg` | `system_state` |
| `/inspection/events`（新） | `GraspEvent.msg`（新） | `grasp_event` |

---

## 6. 并发模型

```
┌──────────────────┐     ┌──────────────────────────────────┐
│ ROS2 Executor    │     │ uvicorn (asyncio 主线程)          │
│ (守护线程)        │     │                                  │
│                  │     │  HTTP handlers (sync, threadpool) │
│ SystemState.msg ─┼──→ StateHub ──→ WebSocket handler      │
│ GraspEvent.msg  ─┼──→ EventHub ──→ WebSocket handler (new)│
└──────────────────┘     └──────────────────────────────────┘
```

- ROS2 executor 在守护线程中运行，处理订阅回调和 service client 完成回调
- uvicorn 异步事件循环在主线程，处理 HTTP 和 WebSocket
- `StateHub` 通过线程安全的 `_LatestQueue(maxsize=1)` 桥接两个线程域
- `RosBridge._call()` 使用 `threading.Lock` 序列化 ROS2 service 调用

---

## 7. 数据模型溯源

`api/models.py` 中的 Pydantic v2 模型**历史上**对应原 `inspection-api/proto/inspection_gateway.proto` 的消息定义。

抓取方向下需要重新梳理（⚠ = 待调整）：

| 历史 Proto 消息 / Pydantic | 抓取方向状态 | 说明 |
|-----------|--------------|------|
| `Pose2D` | 保留 | AGV 位姿与站位 |
| `InspectionTarget` | ⚠ 抓取 V1 不用 | CAD 表面选点，保留供 6D 位姿路径或清理 |
| `InspectionPoint` | ⚠ 不用 | 巡检点位 |
| `InspectionPath` | ⚠ 不用 | 巡检路径 |
| `TaskStatus` | 保留 | 需要扩 `phase` 枚举 + `gripper_status` 字段 |
| `InspectionEvent` | ⚠ 改名/替换 | 改为 `GraspEvent`，字段见 §3 |
| `NavMapInfo` | 保留 | 导航底图 |
| `AgvStatus` / `ArmStatus` | 保留 | 新增 `GripperStatus` |

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
课题:    巡检 → 复合机器人抓取（本仓库/文档的主方向）
```

- `inspection-api` → `api/models.py` (Pydantic) + OpenAPI 自动文档
- `inspection-hmi/GatewayClient.cpp` → `inspection-site/src/api/` (fetch + WebSocket)
- `inspection-hmi/OperatorWindow` → `inspection-site/src/pages/OperatorPage.tsx`
- `inspection-hmi/MainWindow` → `inspection-site/src/pages/EngineerPage.tsx` (抓取方向下 Engineer 的职责更轻：数据集/权重/任务模板管理)
