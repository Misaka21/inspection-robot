# inspection_gateway/CLAUDE.md

本包是 **HMI <-> ROS2 的桥接层**：

`inspection-hmi (Qt/Windows) --gRPC--> inspection_gateway (AGX) --ROS2--> drivers/controllers/planners`

目标：让上位机不需要 ROS2/DDS 环境，仅通过 `inspection-api/proto/inspection_gateway.proto` 的 gRPC 接口完成控制、状态监控、导航底图、媒体回显。

## 1. 包职责与边界

负责：
- 对外实现 gRPC `InspectionGateway`（proto 为准）
- 对内桥接 ROS2 topic/service
- 缓存与落盘：CAD、targets、plan、task、media、results
- 维护稳定的状态流与事件流（支持断线重连 + snapshot）

不负责：
- 设备厂商协议（只能在各 `*_driver` 内）
- 规划/检测算法（只编排调用与数据编排）

## 2. 分层架构（强制，避免堆在一个 main 里）

建议目录（后续实现按此落地）：

- `inspection_gateway/rpc/`：gRPC handler（薄层，只做 pb <-> domain 映射）
- `inspection_gateway/ros/`：ROS2 bridge（clients/subscriptions/publishers）
- `inspection_gateway/store/`：落盘与索引（cad/targets/plan/task/media/results）
- `inspection_gateway/domain/`：纯数据结构与转换（不依赖 ROS/gRPC）

强制约束：
- **禁止** 在 `main.py` 里实现具体业务；`main.py` 只负责进程启动、参数解析、线程模型装配。
- **禁止** 在 gRPC handler 里做阻塞 ROS 调用（需要统一的 worker / async 调度）。

## 3. gRPC <-> ROS2 映射（V1 目标）

详细桥接设计见：`../../docs/INSPECTION_GATEWAY_DESIGN.md`

最小闭环（P0）：
- 控制：`Start/Pause/Resume/Stop/GetTaskStatus`
  - ROS2: `/inspection/start|pause|resume|stop|get_status`
- 状态流：`SubscribeSystemState`
  - ROS2: 订阅 `/inspection/state`
- 导航底图：`GetNavMap`
  - ROS2: 调 `/inspection/agv/get_nav_map`（由 `agv_driver` 提供）
- 媒体：`DownloadMedia`（先把抓拍落盘 + media_id 跑通）

## 4. 并发模型（建议）

推荐双线程域：
- ROS2 executor 线程：处理订阅回调、service client 完成回调
- gRPC 线程池：处理 RPC 与 stream
- 两者通过 thread-safe queue / 原子快照交换数据（避免互相阻塞）

## 5. 文档与 TODO 维护（必须）

- 本仓库根 `TODO.md` 是单一事实来源：任何 gateway 未完成项必须写进 `TODO.md`
- 对外 proto 变化：修改 `inspection-api`，并同步更新：
  - `../../docs/WORKSPACE_OVERVIEW.md`
  - `../../docs/IMPLEMENTATION_STATUS.md`
  - `../../TODO.md`
- 对内 ROS2 契约变化（topic/service）：同步更新：
  - `../../docs/ARCHITECTURE.md`
  - 相关包 `../*/CLAUDE.md`
  - `../../TODO.md`

