# inspection_gateway 设计说明（历史/可选）

当前课题方向已改回**大型工件视觉检测/巡检**，且当前系统演示不依赖网页前端。因此 `inspection_gateway` 不再作为论文主线模块。

## 当前定位

- 历史 FastAPI REST/WS 网关。
- 保留源码，避免破坏已有工程。
- 当前不作为任务启动、状态展示、结果回看的必需链路。
- 当前论文默认通过 ROS2 CLI、RViz、rqt_image_view 和实验脚本完成控制与记录。

## 若未来恢复网页

需要重新整理：

- REST API 模型与 `inspection_interface` 的映射。
- `/inspection/state` 到 WebSocket 的状态推送。
- 缺陷检测事件 `inspection_event`。
- 图像媒体落盘与 `GET /tasks/{id}/captures`。
- 导航地图 `GET /nav/map`。

恢复网页前，应先更新：

- `README.md`
- `TODO.md`
- `docs/WORKSPACE_OVERVIEW.md`
- `docs/ARCHITECTURE.md`
- `docs/IMPLEMENTATION_STATUS.md`
- `src/inspection_gateway/CLAUDE.md`

