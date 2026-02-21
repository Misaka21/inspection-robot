# inspection_gateway/CLAUDE.md

本包是 **Web HMI <-> ROS2 的桥接层**：

`浏览器 (任意设备) --REST/WebSocket--> inspection_gateway (AGX) --ROS2--> drivers/controllers/planners`

目标：让上位机不需要 ROS2/DDS 环境，仅通过 HTTP REST + WebSocket 完成控制、状态监控、导航底图、媒体回显。前端代码在独立的 `inspection-site` 仓库中开发，构建产物部署到 `frontend/dist/` 由 FastAPI 托管。

## 1. 包职责与边界

负责：
- 对外提供 REST API（FastAPI，`/api/v1/...`）和 WebSocket（`/ws`）
- 对内桥接 ROS2 topic/service
- 缓存与落盘：CAD、targets、plan、task、media、results
- 托管前端静态文件（`frontend/dist/`）
- 维护稳定的状态流与事件流（WebSocket 支持断线重连 + 初始快照）

不负责：
- 设备厂商协议（只能在各 `*_driver` 内）
- 规划/检测算法（只编排调用与数据编排）
- 前端源码开发（在 `inspection-site` 仓库）

## 2. 分层架构

```
inspection_gateway/
├── main.py              # 进程入口: ROS2 线程 + uvicorn
├── api/                 # REST + WebSocket 层
│   ├── app.py           # FastAPI 工厂 (create_app)
│   ├── deps.py          # 依赖注入
│   ├── models.py        # Pydantic v2 数据模型 (API 契约)
│   ├── routes/          # REST 端点
│   │   ├── tasks.py     # POST/GET /api/v1/tasks/...
│   │   ├── nav.py       # GET /api/v1/nav/map
│   │   ├── media.py     # GET /api/v1/media/{id}
│   │   ├── cad.py       # POST /api/v1/cad/upload
│   │   ├── targets.py   # POST /api/v1/targets
│   │   ├── plans.py     # POST/GET /api/v1/plans
│   │   └── captures.py  # GET /api/v1/tasks/{id}/captures
│   └── ws/
│       ├── handler.py   # WebSocket /ws 端点
│       └── protocol.py  # 消息信封定义
├── ros/                 # ROS2 bridge (不改)
│   ├── bridge.py        # RosBridge (service clients)
│   └── state_hub.py     # StateHub (线程安全 pub/sub)
├── store/               # 本地持久化 (不改)
│   ├── cad_store.py
│   └── media_store.py
├── domain/              # 纯数据 + 转换
│   ├── converters.py    # ROS msg -> Pydantic model
│   └── runtime.py       # GatewayRuntime (task_id 等运行时状态)
├── frontend/            # 前端构建产物 (由 inspection-site 生成)
│   └── dist/            # npm run build 输出, FastAPI 静态托管
```

强制约束：
- **禁止** 在 `main.py` 里实现具体业务；只负责进程启动、参数解析、线程模型装配。
- route handler 保持薄：同步等待 ROS srv，不做复杂业务逻辑。

## 3. REST API 端点

| HTTP    | Path                            | 说明                |
|---------|---------------------------------|---------------------|
| POST    | `/api/v1/cad/upload`           | multipart/form-data |
| POST    | `/api/v1/targets`              | JSON body           |
| POST    | `/api/v1/plans`                | 返回 plan_id        |
| GET     | `/api/v1/plans/{plan_id}`      |                     |
| POST    | `/api/v1/tasks`                | 返回 task_id        |
| POST    | `/api/v1/tasks/{id}/pause`     |                     |
| POST    | `/api/v1/tasks/{id}/resume`    |                     |
| POST    | `/api/v1/tasks/{id}/stop`      |                     |
| GET     | `/api/v1/tasks/{id}/status`    | 一次性查询          |
| GET     | `/api/v1/nav/map`              |                     |
| GET     | `/api/v1/tasks/{id}/captures`  |                     |
| GET     | `/api/v1/media/{media_id}`     | StreamingResponse   |
| WS      | `/ws`                          | 实时状态推送        |

OpenAPI 文档自动生成: `http://localhost:8080/docs`

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

## 5. 并发模型

双线程域：
- ROS2 executor 线程（守护线程）：处理订阅回调、service client 完成回调
- uvicorn 异步事件循环（主线程）：处理 HTTP 请求与 WebSocket
- 两者通过 StateHub 的线程安全队列交换数据

## 6. 依赖

- `rclpy`, `inspection_interface`
- `fastapi>=0.100.0`, `uvicorn[standard]>=0.23.0`, `pydantic>=2.0.0`, `python-multipart`

## 7. 开发与部署

```bash
# 开发 (两个终端)
# Terminal 1: FastAPI + ROS2
python -m inspection_gateway.main --port 8080

# Terminal 2: 前端开发服务器 (在 inspection-site 目录)
cd inspection-site && npm run dev  # :5173, 代理 /api 和 /ws 到 :8080

# 部署
cd inspection-site && npm run build  # 输出到 inspection_gateway/frontend/dist/
python -m inspection_gateway.main --port 8080  # 一个端口搞定
```
