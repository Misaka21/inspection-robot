# TODO（inspection-robot）

本文件维护“还要做什么”的**单一事实来源**（Single Source of Truth）。  
任何跨包接口/数据流的修改，都必须同步更新本文档与相关 `docs/*` / `src/*/CLAUDE.md`，避免口径漂移。

相关参考：
- 端到端约定：`docs/WORKSPACE_OVERVIEW.md`
- 落地缺口清单：`docs/IMPLEMENTATION_STATUS.md`
- 对外契约：`../inspection-api/proto/inspection_gateway.proto`

## P0（必须，先把效果跑出来）

- [x] 实现 `inspection_gateway`（gRPC server）最小可用闭环（启动/ROS bridge/落盘/cache/proto 加载）
- [x] 支持控制面：`Start/Pause/Resume/Stop/GetTaskStatus/SubscribeSystemState`
- [x] 支持导航底图：`GetNavMap`（网关侧已实现；依赖 ROS `agv_driver/get_nav_map`）
- [x] 支持媒体下载：`DownloadMedia`（网关侧 `media_store` 已实现）
- [ ] `agv_driver` 实现 `get_nav_map` service server（优先返回 map_id/resolution/origin/thumbnail；底图可渐进增强）
- [ ] `task_coordinator` 补齐“按 waypoint 执行”的真实下发：AGV goal -> 等到位停止 -> 机械臂动作 -> 触发相机/检测 -> 推进进度
- [ ] 取图链路最小化可回显：抓拍图落盘成可下载 `media_id` + 能被 HMI 列表/回看（`ListCaptures` + thumbnail）
- [ ] TF/标定口径统一并可验证：`map->base_link->arm_base->tcp->camera`（把当前 `tool0`/URDF frame 不一致的问题一次性定死）

## P1（规划与约束，对齐 inspection-api 语义）

- [ ] `path_planner` 从 Trigger 骨架升级为“输入 targets+capture_config 输出 InspectionPath”的 ROS2 srv（网关调用）
- [ ] 引入 `CaptureConfig.focus_distance_m` + `max_tilt_from_normal_deg` 的约束进入采样/IK 过滤（为后续逆解做铺垫）
- [ ] `task_coordinator` 与 `inspection_interface` 对齐：SystemState 能无损映射到 gRPC `TaskStatus`

## P2（结果与事件流）

- [ ] 新增 `/inspection/events`（结构化事件：CAPTURED/DEFECT_FOUND/ERROR...），网关映射到 `SubscribeInspectionEvents`
- [ ] 引入 `capture_manager`（或同等模块）：图像落盘/索引、thumbnail 生成、按 task/point_id 查询
- [ ] 支持 `ListCaptures(task_id, point_id)` 与历史回看

## P3（工程化）

- [ ] 为关键模块补单测：`agv_map_parser`、`task_coordinator` 核心状态机（core）等
- [ ] `inspection_supervisor` 改为订阅 `/inspection/agv/status` + `/inspection/arm/status` + `/inspection/state`，并把健康/告警换成结构化 msg
- [ ] Bringup 参数体系收敛：所有 topic/service 名称与 namespace 通过 launch 控制，不在代码里散落字符串
