# inspection_gateway 设计（HMI <-> ROS2 桥接）

目标：上位机 `inspection-hmi` 不需要 ROS2 环境，只通过 gRPC 调用 `inspection_gateway`；`inspection_gateway` 运行在机器人端（AGX/工控机），在同一进程内访问 ROS2 topic/service，并对外提供 `inspection-api/proto/inspection_gateway.proto` 定义的能力。

代码落点：
- 本仓库已建立 ROS2 包骨架：`src/inspection_gateway/`
- 网关实现应放在该包内（避免散落脚本）

## 1. 部署与通信边界

- 部署位置：机器人端（与 ROS2 同机或同容器）
- HMI 网络：HMI 与 gateway 在同一局域网，HMI 只需要能访问 gateway 的 gRPC 端口
- 外部协议：gRPC（必要），可选 HTTP（仅用于 `MediaRef.url` 直链下载）
- 内部协议：ROS2（topic/service），由 gateway 屏蔽 ROS2 细节

推荐默认端口（可配置，不写死）：
- gRPC：`50051`
- HTTP（可选）：`8080`

## 2. Gateway 的职责（必须）

1. **协议适配**：实现 gRPC `InspectionGateway`，保持对外字段/语义稳定（以 `inspection-api` 为准）
2. **ROS2 桥接**：把 gRPC 调用映射为 ROS2 service call / topic subscribe / topic publish
3. **缓存与落盘**：
   - CAD 上传文件、targets、plan、task 元数据
   - 地图底图/缩略图缓存
   - 抓拍图/缩略图/缺陷结构化结果落盘与索引
4. **状态与事件流**：
   - `SubscribeSystemState`：稳定推送（断线可重连；可携带 snapshot）
   - `SubscribeInspectionEvents`：抓拍/缺陷/告警事件（面向 UI 时间轴）

不做（V1 不强制）：
- 复杂权限/账号系统
- TLS/证书部署（需要时再加）

## 3. gRPC <-> ROS2 映射（建议对内契约）

说明：gRPC 需要完整语义，但 ROS2 侧可能还没实现齐全。gateway 允许“用本地 store 补字段”，但要在 `TODO.md` 里记录缺口。

### 3.1 控制面

- `StartInspection(plan_id)` -> ROS2 `/inspection/start` (`inspection_interface/srv/StartInspection`)
- `PauseInspection(task_id)` -> ROS2 `/inspection/pause`
- `ResumeInspection(task_id)` -> ROS2 `/inspection/resume`
- `StopInspection(task_id)` -> ROS2 `/inspection/stop`
- `GetTaskStatus(task_id)` -> ROS2 `/inspection/get_status` + gateway 本地 `task_store/plan_store` 合并补齐字段
- `SubscribeSystemState(task_id)` -> 订阅 ROS2 `/inspection/state` (`inspection_interface/msg/SystemState`) 并映射为 gRPC `TaskStatus`

### 3.2 规划面（后续补齐）

- `SetInspectionTargets`：
  - V1 可先由 gateway 落盘（`target_store`），并作为 `PlanInspection` 的输入
  - 后续建议增加 ROS2 `SetTargets.srv`（由 planner/coordinator 侧消费）
- `PlanInspection`：
  - 目标：返回 gRPC `InspectionPath`
  - 建议：planner 提供 ROS2 `PlanInspection.srv`（返回 waypoints/stats），gateway 只做参数/数据编排与 plan 存储

### 3.3 导航地图

- `GetNavMap(map_id="")` -> ROS2 `/inspection/agv/get_nav_map` (`inspection_interface/srv/GetNavMap`)
- gateway 对外返回 `NavMapInfo` 时，建议把 `image_data` 转成 `MediaRef.media_id`，并同时返回 `thumbnail_jpeg`（UI 预览用）

`map_id` 建议格式（稳定可缓存）：
- `agv:{map_name}:{map_md5}`

### 3.4 媒体与结果

- `DownloadMedia(media_id)`：
  - gateway 从本地 `media_store` 读取文件并分 chunk 输出
- `ListCaptures(task_id, point_id)`：
  - gateway 从 `result_store` 读取索引并返回结构化记录

## 4. 数据流（端到端）

```mermaid
flowchart LR
  HMI["inspection-hmi (Qt, Windows)"]
  GW["inspection_gateway (gRPC server, AGX)"]
  ROS["ROS2 graph"]

  HMI -->|"gRPC"| GW
  GW -->|"ROS2 service/topic"| ROS
```

## 5. 代码结构规划（避免把逻辑堆在一个文件）

推荐分层（不超过 4 层）：

- `rpc/`：gRPC handler（薄层，只做 pb <-> domain 映射与错误码）
- `ros/`：ROS2 bridge（service client、topic subscriber、topic publisher）
- `store/`：落盘与索引（cad/targets/plan/task/media/results）
- `domain/`：纯数据结构与映射函数（不依赖 ROS/gRPC）

建议的核心模块：
- `NavMapService`：ROS `GetNavMap` + 缓存 + media_id 管理
- `TaskService`：Start/Pause/Resume/Stop/GetStatus + task_id 生成 + 合并 store 字段
- `StateStreamer`：订阅 `/inspection/state`，做节流/丢帧保护，把快照推到多个 gRPC stream
- `MediaStore`：文件落盘、sha256、chunk 读取
- `CaptureStore/ResultStore`：capture_id/point_id 索引与查询

## 6. 并发模型（V1 重点）

约束：ROS2 回调与 gRPC stream 都可能高频，不能互相阻塞。

推荐做法（任选其一，建议 A）：

- A：gateway 进程内双线程域
  - 线程 1：ROS2 executor（订阅/回调）
  - 线程 2：gRPC server（处理请求/stream）
  - 两者通过 thread-safe queue/atomic snapshot 交换数据

- B：gateway 单线程 + 异步事件循环（复杂度更高，慎用）

## 7. ID 与落盘约定（建议）

建议全都用 UUID（或 sha256）保证重连可恢复：
- `model_id`：sha256(CAD bytes) 或 UUID
- `plan_id`：UUID
- `task_id`：UUID
- `capture_id`：UUID
- `media_id`：UUID 或 sha256(file bytes)

落盘根目录（可配置）：
- `/var/lib/inspection_gateway/`
  - `cad/`
  - `plans/`
  - `tasks/`
  - `media/`
  - `results/`

## 8. 与 HMI 的连接约定（建议）

HMI 需要实现：
- 连接配置（host/port/timeout）
- 断线重连（指数退避）
- stream 的“掉线重订阅”（`include_snapshot=true` 先拿快照再追实时）

## 9. 文档与 TODO 维护（必须）

- 对外 proto 变化：更新 `inspection-api`，并同步更新本仓库 `docs/WORKSPACE_OVERVIEW.md` 与 `TODO.md`
- 对内 ROS2 契约变化（topic/service）：同步更新 `docs/ARCHITECTURE.md`、相关包 `src/*/CLAUDE.md` 与 `TODO.md`
