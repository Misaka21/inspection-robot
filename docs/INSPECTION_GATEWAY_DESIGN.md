# inspection_gateway 设计（HMI ↔ ROS2 桥接）

> 详细实现状态、代码结构、端点清单见 `src/inspection_gateway/CLAUDE.md`。本文档侧重**设计决策与架构概览**。
>
> **方向说明**：当前课题是"复合机器人抓取"；网关名和 `/inspection/*` 路径前缀是历史遗留（巡检命名）。对外 API 语义需要从"巡检任务"调整为"抓取任务"，见 §3。

## 架构演进

```
原架构:  inspection-api (proto)  ← gRPC →  inspection-hmi (Qt C++)  →  inspection_gateway (gRPC server, :50051)
现架构:  inspection-site (React) ← REST/WS → inspection_gateway (FastAPI, :8080) → ROS2
```

- `inspection-api` + `inspection-hmi` 已归档，不再维护
- 外部协议从 gRPC 迁移为 REST + WebSocket（降低前端接入门槛，支持浏览器直连）
- 数据契约从 `.proto` 迁移为 `api/models.py`（Pydantic v2），OpenAPI 文档自动生成

## 1. 部署与通信边界

- 部署位置：机器人端（Jetson Orin AGX），与 ROS2 同进程
- 前端访问：浏览器通过局域网访问 `http://<robot_ip>:8080`
- 外部协议：REST（`/api/v1/...`）+ WebSocket（`/ws`）
- 内部协议：ROS2（topic/service），由 gateway 屏蔽 ROS2 细节

端口（可配置）：`8080`（REST + WS + 前端静态文件，统一端口）

## 2. Gateway 的职责

1. **协议适配**：实现 REST API + WebSocket 端点，保持对外字段/语义稳定（以 `api/models.py` 为准）
2. **ROS2 桥接**：把 HTTP 请求映射为 ROS2 service call，把 ROS2 topic 订阅映射为 WebSocket 推送
3. **存储**：
   - CAD 文件（`CadStore`，SHA256 内容寻址；6D 位姿升级路径会用到）
   - 媒体资源（`MediaStore`，SHA256 内容寻址）
   - Task 运行时状态（`GatewayRuntime`，内存缓存）
4. **前端托管**：`frontend/dist/` 由 FastAPI 静态文件中间件托管
5. **状态推送**：WebSocket 推送 `system_state`（来自 ROS2 `SystemState` topic）和 `grasp_event`（待实现）

不做（V1）：
- 复杂权限/账号系统
- TLS/证书（需要时再加）
- 持久化数据库（当前用内存 + 文件系统）

## 3. REST/WS ↔ ROS2 映射（抓取方向调整）

### 3.1 控制面（抓取任务）

| HTTP 端点 | 当前 ROS2 服务 | 目标 ROS2 服务 | 说明 |
|-----------|----------------|----------------|------|
| `POST /api/v1/tasks` | `/inspection/start` (`StartInspection.srv`) | `/inspection/start` (`StartGrasp.srv`) | 请求体需加 `object_class`/`pick_config`/`place_config` |
| `POST /api/v1/tasks/{id}/pause` | `/inspection/pause` | 不变 | |
| `POST /api/v1/tasks/{id}/resume` | `/inspection/resume` | 不变 | |
| `POST /api/v1/tasks/{id}/stop` | `/inspection/stop` | 不变 | |
| `GET /api/v1/tasks/{id}/status` | `/inspection/get_status` | 不变（但 phase 枚举需扩） | |

### 3.2 规划面（巡检遗留，抓取场景基本不用）

| HTTP 端点 | 抓取场景需求 | 处理建议 |
|-----------|-------------|----------|
| `POST /api/v1/targets` | 不再需要（CAD 表面选点是巡检概念） | 保留端点接收但不使用；或后续清理 |

### 3.3 导航地图与媒体

| HTTP 端点 | ROS2 服务 | 说明 |
|-----------|----------|------|
| `GET /api/v1/nav/map` | `/inspection/agv/get_nav_map` (`GetNavMap.srv`) | 图片写入 MediaStore，返回 media_id |
| `GET /api/v1/media/{media_id}` | — | 从 MediaStore 读取，StreamingResponse 256KB 分块 |
| `GET /api/v1/tasks/{id}/captures` | (网关内处理) | 抓取场景需要（观察图/抓取瞬间图），目前桩代码 |

### 3.4 WebSocket 推送

| WS 消息类型 | ROS2 来源 | 实现状态 | 抓取方向 |
|------------|----------|---------|---------|
| `system_state` | `/inspection/state` (`SystemState.msg`) | **已实现** | 需扩充 phase 枚举（新增抓取阶段） |
| `grasp_event` **[新]** | `/inspection/events` (`GraspEvent.msg`，待新增) | **未实现** | 替代原 `inspection_event` 语义 |
| `ping` / `pong` | — | **已实现** | 不变 |

## 4. 数据流

```mermaid
flowchart LR
  FE["inspection-site (React, 浏览器)"]
  GW["inspection_gateway (FastAPI, AGX :8080)"]
  ROS["ROS2 graph
  (task_coordinator,
   drivers, controllers,
   grasp_perception,
   gripper_driver...)"]

  FE -->|"REST /api/v1/*"| GW
  FE <-->|"WebSocket /ws"| GW
  GW -->|"ROS2 service call"| ROS
  ROS -->|"ROS2 topic"| GW
```

## 5. 并发模型

```
┌──────────────────┐     ┌──────────────────────────────────┐
│ ROS2 Executor    │     │ uvicorn (asyncio 主线程)          │
│ (守护线程)        │     │                                  │
│                  │     │  HTTP handlers (sync, threadpool) │
│ SystemState.msg ─┼──→ StateHub ──→ WebSocket handler      │
│  callback        │     │                                  │
│ GraspEvent.msg  ─┼──→ EventHub ──→ WebSocket handler      │
│  (待新增)         │     │                                  │
└──────────────────┘     └──────────────────────────────────┘
```

- ROS2 executor 在守护线程中运行（订阅回调、service client 完成回调）
- uvicorn 异步事件循环在主线程（HTTP、WebSocket）
- `StateHub` 通过线程安全的 `_LatestQueue(maxsize=1)` 桥接两个线程域
- `RosBridge._call()` 使用 `threading.Lock` 序列化 ROS2 service 调用

## 6. ID 与存储约定

| ID 类型 | 生成方式 | 说明 |
|---------|---------|------|
| `model_id` | SHA256(CAD bytes) | 内容寻址，相同文件同 ID（6D 路径使用） |
| `task_id` | UUID | 每次启动任务生成新 ID |
| `media_id` | SHA256(file bytes) | 内容寻址（观察图/抓取图/地图底图都复用） |

存储根目录（可配置）：
- `cad_store/`：CAD 模型文件
- `media_store/`：地图图片、抓拍图片等媒体

## 7. 数据模型（三层一致性）

```
api/models.py (Pydantic v2)  ←→  inspection_interface (ROS2 msg/srv)  ←→  inspection-site/src/api/types.ts
```

修改任一层的数据模型时，必须同步更新其他两层。`api/models.py` 是 API 契约的**唯一事实来源**。

抓取方向的数据模型调整（待落地）：

| Pydantic 模型 | 调整 |
|--------------|------|
| `StartInspection` 请求 | 改名 `StartGrasp` / 或在 `StartInspection` 增加 `object_class/pick/place` 字段 |
| `TaskStatus.phase` | 扩充枚举：`NAV_TO_PICK/ARM_TO_OBSERVE/CAPTURE_RGBD/PERCEIVE/TRANSFORM_IK/PRE_GRASP/GRASP/LIFT/NAV_TO_PLACE/PLACE/HOME/RECOVERY` |
| `InspectionEvent` | 改名或新增 `GraspEvent`：`type` 枚举改为 `OBSERVED/PERCEIVED/GRASPED/PLACED/FAILED`；`defects` 字段去掉，新增 `grasp_pose/media_id/message` |
| `NavMapInfo` | 不变 |
| `AgvStatus` / `ArmStatus` | 不变；**新增** `GripperStatus` |

## 8. 文档与 TODO 维护（必须）

- REST/WS API 变化：更新 `src/inspection_gateway/CLAUDE.md`、`docs/WORKSPACE_OVERVIEW.md`、`TODO.md`
- ROS2 契约变化（topic/service）：同步更新 `docs/ARCHITECTURE.md`、相关包 `src/*/CLAUDE.md`、`TODO.md`
