# TODO（inspection-robot）

本文件维护"还要做什么"的**单一事实来源**（Single Source of Truth）。
任何跨包接口/数据流的修改，都必须同步更新本文档与相关 `docs/*` / `src/*/CLAUDE.md`，避免口径漂移。

相关参考：
- 端到端约定：`docs/WORKSPACE_OVERVIEW.md`
- 落地缺口清单：`docs/IMPLEMENTATION_STATUS.md`
- 对外 API 契约：`src/inspection_gateway/inspection_gateway/api/models.py`

## P0（必须，先把效果跑出来）

- [x] 实现 `inspection_gateway`（FastAPI REST/WS server）最小可用闭环（启动/ROS bridge/落盘/cache）
- [x] 支持控制面：`POST /tasks` (Start) + `pause/resume/stop` + `GET /tasks/{id}/status` + `WS system_state`
- [x] 支持导航底图：`GET /nav/map`（网关侧已实现；依赖 ROS `agv_driver/get_nav_map`）
- [x] 支持媒体下载：`GET /media/{media_id}`（网关侧 `MediaStore` 已实现）
- [x] 新增 `inspection_sim`：无硬件端到端联调
- [x] `task_coordinator` 状态机改为配置驱动（YAML 站位序列）：MOVING_TO_STATION → ARM_PRESET → DEPTH_ADJUST → CAPTURING
- [x] 删除 `pose_detector` 和 `path_planner`（需求简化，不再需要 6D 位姿推断和联合优化路径规划）
- [x] 删除 gateway 的 plans 端点（巡检任务由 YAML 配置驱动，无需动态规划）
- [ ] `agv_driver` 实现 `get_nav_map` service server（优先返回 map_id/resolution/origin/thumbnail；底图可渐进增强）
  - 仿真端 `fake_agv` 已提供，真机端需封装厂商 TCP API（1300/4011/1513）
- [ ] 取图链路最小化可回显：抓拍图落盘成可下载 `media_id` + 能被前端列表/回看（`GET /tasks/{id}/captures` + thumbnail）
- [ ] TF/标定口径统一并可验证：`map→base_link→arm_base→tool0→hikvision_frame`（补充 `base_link→arm_base` 静态 TF）
- [ ] `defect_detector` 填充实际检测算法（当前为骨架代码）
- [ ] 深度补偿逻辑真机验证：确认 RealSense 深度图 ROI 中心测距精度满足 ±2cm 容差

## P1（功能完善）

- [ ] `task_coordinator` 订阅 `defect_detector` 结果 topic（当前只调 Trigger 看 bool，缺陷详情丢失）
- [ ] `inspection_stations.yaml` 真机标定：填入实际 AGV 站位坐标和机械臂关节角
- [ ] `sim_system.launch.py` 清理遗留的 gRPC 参数（`grpc_port` / `--grpc-port`）

## P2（结果与事件流）

- [ ] 新增 `/inspection/events`（结构化事件：CAPTURED/DEFECT_FOUND/ERROR...），网关映射到 WebSocket `inspection_event`
- [ ] 引入 `capture_manager`（或同等模块）：图像落盘/索引、thumbnail 生成、按 task/point_id 查询
- [ ] 支持 `GET /tasks/{id}/captures` 与历史回看

## P3（工程化）

- [ ] 为关键模块补单测：`agv_map_parser`、`task_coordinator` 核心状态机（core）等
- [ ] `inspection_supervisor` 改为订阅 `/inspection/agv/status` + `/inspection/arm/status` + `/inspection/state`，并把健康/告警换成结构化 msg
- [ ] Bringup 参数体系收敛：所有 topic/service 名称与 namespace 通过 launch 控制，不在代码里散落字符串
- [ ] `inspection_supervisor` 补充 launch 文件并纳入 `system.launch.py`

## P3（架构重构，各包 CLAUDE.md 建议）

- [ ] `task_coordinator` 拆分 `CoordinatorCore`（无 ROS 依赖）+ `InterlockPolicy` + `RosAdapter`
- [ ] `arm_controller` 拆分 `MoveItFacade` + `TrajectoryExecutor` + `DriverBridge`；解决回调阻塞问题
- [ ] `hikvision_driver` 拆分 `HkSdkSession` + `GrabWorker` + `MonitorWorker` + `RosAdapter`
- [ ] `defect_detector` 新增 `DefectModel` + `Preprocess` + `Postprocess`
- [ ] `inspection_supervisor` 拆分 `LivenessWatcher` + `ErrorCodeWatcher` + `InterlockWatcher`
- [ ] 统一 `arm_driver` / `arm_controller` / `hikvision_driver` / `inspection_supervisor` 命名规范为 `_prefix`
