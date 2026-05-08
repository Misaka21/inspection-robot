# inspection_gateway — CLAUDE.md

本包是历史 FastAPI REST/WS 网关。当前课题已改回**大型工件视觉检测/巡检**，且用户确认当前没有网页前端，因此本包不作为论文和演示主线。

## 当前定位

- 保留源码，避免破坏历史工程。
- 不作为当前巡检系统的必要链路。
- 不要求与 `task_coordinator` 的最新巡检接口同步优先演进。
- 若后续恢复网页/HMI，再重新整理 API 模型、WebSocket 事件和媒体回看。

## 当前主线替代方式

巡检任务控制与结果观察默认通过：

- `ros2 launch`
- `ros2 service call`
- `ros2 topic echo`
- RViz / rqt_image_view
- 实验脚本保存图像与 CSV

## 维护规则

- 当前不要为了论文主线新增 REST/WS 功能。
- 如果修改本包，必须确认不会影响主线 ROS2 包构建。
- 如果未来恢复网页功能，先更新 `docs/WORKSPACE_OVERVIEW.md`、`docs/ARCHITECTURE.md` 和 `TODO.md`，再改代码。

