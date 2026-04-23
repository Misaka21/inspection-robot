# TODO（inspection-robot，抓取系统）

本文件维护"还要做什么"的**单一事实来源**（Single Source of Truth）。
任何跨包接口/数据流的修改，都必须同步更新本文档与相关 `docs/*` / `src/*/CLAUDE.md`，避免口径漂移。

相关参考：
- 端到端约定：`docs/WORKSPACE_OVERVIEW.md`
- 落地缺口清单：`docs/IMPLEMENTATION_STATUS.md`
- 对外 API 契约：`src/inspection_gateway/inspection_gateway/api/models.py`

> **课题方向更新（2026-04-23）**：从"大型工件视觉检测（巡检）"调整为"复合移动机器人抓取"。
> - 新 pipeline：AGV 到点 → 手眼相机拍 RGBD → 感知出抓取位姿 → MoveIt 规划 → 气动夹爪抓 → 放置
> - 相机：RealSense D435，**eye-in-hand**（装机械臂末端）
> - 夹爪：气动二指夹爪（通过 `dio_driver` 的 DO 线驱动电磁阀）
> - 算力：Jetson Orin AGX 64G（不选 GraspNet/AnyGrasp；已知工业件用 YOLOv8+深度 或 6D 位姿估计）
> - 历史巡检相关 P0/P1/P2 任务（8 点位循环、defect_detector 填充等）已**归档**到文末"附录 A：巡检场景遗留任务（不再推进）"

---

## P0（必须，把抓取 demo 跑出来）

### 基础设施
- [ ] **`base_link → arm_base` 静态 TF**：机械臂安装位姿标定，yaml 存在 `inspection_bringup/config/mount_calib.yaml`，由 bringup 发布
- [ ] **`tool0 → camera_link` 手眼标定**：用 `easy_handeye2` + ChArUco 完成；结果存 `inspection_bringup/config/handeye_calib.yaml`
- [ ] **`tool0 → gripper_tip` 静态 TF**：气动夹爪 offset，量一次写死在 yaml
- [ ] `inspection_bringup/drivers.launch.py` 替换 `tool0 → hikvision_frame` 为 `tool0 → camera_link` + `tool0 → gripper_tip`
- [ ] `inspection_bringup/system.launch.py` 纳入 `realsense_driver` + `dio_driver` + `gripper_driver` + `grasp_perception` + `arm_controller` + `task_coordinator`

### 夹爪（新增包 `gripper_driver`）
- [ ] 新建 `gripper_driver`（Python 或 C++ 都行，先简单）：封装 `dio_driver/set_output` 为 `/inspection/gripper/open`、`/inspection/gripper/close`（`std_srvs/srv/Trigger`）
- [ ] `gripper_driver` 订阅 `dio/status`，发布 `/inspection/gripper/status`（`std_msgs/Bool` 或新增 `GripperStatus.msg`）
- [ ] 确认气动电磁阀接线到的 DO 线号（默认假设 DO0=夹紧，DO1=松开，需真机确认），写到 yaml

### 感知（新增包 `grasp_perception`）
- [ ] 新建 `grasp_perception`（Python）：订阅 RealSense `color/depth/camera_info`，提供 `srv/PerceiveGrasp` 服务
- [ ] 路径 A（先跑通）：YOLOv8 目标检测 + 深度取 bbox 中心 Z + 固定抓取姿态（类别到姿态的 yaml 查表）
- [ ] 数据集准备：每类工业件 100–300 张标注图（YOLO 格式），训练 YOLOv8n
- [ ] 路径 B（加分项，可选）：集成 FoundationPose 或 MegaPose，用 CAD 做 6D 位姿估计 + 预标注抓取点

### 接口（`inspection_interface`）
- [ ] 新增 `srv/PerceiveGrasp.srv`：request `{string object_class(可选)}`，response `{bool success; string error; GraspPose[] candidates; string frame_id}`
- [ ] 新增 `msg/GraspPose.msg`：`{geometry_msgs/Pose pose; float64 width_m; float64 score}`
- [ ] 新增 `srv/StartGrasp.srv`（或沿用 `StartInspection.srv` 改语义）：request `{string task_name; string object_class(可选); bool dry_run}`，response `{bool success; string task_id}`
- [ ] 新增 `msg/GripperStatus.msg`：`{builtin_interfaces/Time stamp; bool is_closed; bool moving}`
- [ ] `msg/SystemState.msg` 新增抓取阶段枚举（PICK/OBSERVE/CAPTURE/PERCEIVE/PRE_GRASP/GRASP/LIFT/PLACE…）

### 任务编排（`task_coordinator`）
- [ ] 状态机重写：从"8 点位巡检循环"→"单次抓取 pipeline"（IDLE → NAV_TO_PICK → ARM_TO_OBSERVE → CAPTURE_RGBD → PERCEIVE → TRANSFORM_IK → PRE_GRASP → GRASP → CLOSE_GRIPPER → LIFT → NAV_TO_PLACE → PLACE → OPEN_GRIPPER → HOME → IDLE）
- [ ] YAML 配置格式改为抓取任务（见 `src/task_coordinator/CLAUDE.md` §4）：`pick` / `place` 两个点位 + 观察位 + 目标类别
- [ ] 调用链：`PerceiveGrasp` service → TF 变换 → `arm_controller/move_to_pose`（Pilz LIN）→ `gripper_driver/close` → 后续步骤
- [ ] 删除/替换 DEPTH_ADJUST 阶段（巡检专用），改为"感知+可达性检查"前置
- [ ] RECOVERY 逻辑：感知无目标 / IK 不可达 / 抓取失败（夹爪力反馈或视觉复核）→ 重拍 N 次

### 控制（`arm_controller`）
- [ ] 引入 Pilz industrial motion planner（MoveIt plugin `pilz_industrial_motion_planner`）做 LIN 直线插补，用于 `PRE_GRASP → GRASP` 段
- [ ] 工作台/AGV 本体加入 MoveIt PlanningScene 碰撞模型（避免撞底盘）
- [ ] `arm_controller/move_to_pose` 服务支持 `planner_id` 参数（OMPL 或 PILZ_LIN）

### 网关（`inspection_gateway`）
- [ ] REST/WS 语义从"巡检任务"调整为"抓取任务"：
  - `POST /api/v1/tasks` 请求体改为 `StartGraspRequest`（含 `object_class` / `pick_config` / `place_config`）
  - `TaskStatus` 的 phase 改为抓取阶段枚举
  - `InspectionEvent` → `GraspEvent`（PERCEIVED / GRASPED / DROPPED / FAILED）
- [ ] 删除/禁用 `/cad/upload` 和 `/targets` 路由（抓取场景不需要 CAD 表面选点）；或标注为"保留以备 6D 位姿路径使用"
- [ ] `/nav/map` 保留（AGV 有站位可视化仍需要）

### AGV（`agv_driver`）
- [ ] 真机端实现 `get_nav_map` service server（仿真端 `fake_agv` 已有）：封装 1300/4011/1513 TCP API

---

## P1（功能完善）

- [ ] `task_coordinator` 订阅 `grasp_perception/result` topic（除了 service 调用外，保留一路结果流供可视化/复盘）
- [ ] 抓取失败检测：夹爪闭合后等 300ms 读 DI（气压/光电）或二次观察 RGB，判定是否真的夹到
- [ ] `inspection_supervisor` 健康监控改为订阅结构化 status：`/inspection/agv/status`、`/inspection/arm/status`、`/inspection/gripper/status`、`/inspection/state`
- [ ] 观察位自动选择：给定目标类别 → 从观察位候选集里挑"无自遮挡 + 可达 + 视野最好"的
- [ ] RealSense 时间同步：`aligned_depth_to_color` 与 `color/image_raw` 的 stamp 偏差要在 `grasp_perception` 里检查
- [ ] `grasp_perception` 可视化：发布 `PoseArray` 抓取候选 + 标注图像（rviz2 调试用）

---

## P2（结果与事件流）

- [ ] 新增 `/inspection/events` topic（结构化事件：OBSERVED / PERCEIVED / GRASPED / DROPPED / FAILED…），网关映射到 WebSocket `grasp_event`
- [ ] 抓拍落盘：`capture_manager` 或等效模块把观察图 / 抓取瞬间图落盘成 `media_id` 供上位机回看
- [ ] 支持 `GET /tasks/{id}/captures` 与历史任务回看

---

## P3（工程化）

- [ ] 手眼标定工具链文档：`docs/HANDEYE_CALIB.md`（怎么采集、怎么跑、精度验证）
- [ ] `grasp_perception` 模型管理：模型文件放 `share/` 外的统一位置，通过参数切换 YOLO/FoundationPose
- [ ] 为关键模块补单测：`agv_map_parser`、`task_coordinator` 核心状态机、`grasp_perception` 的 TF 变换/IK 筛选
- [ ] Bringup 参数体系收敛：所有 topic/service 名称与 namespace 通过 launch 控制，不在代码里散落字符串
- [ ] 统一命名规范（`_prefix`）：`arm_driver` / `arm_controller` / `inspection_supervisor` 等历史违规处
- [ ] `task_coordinator` 拆分 `CoordinatorCore`（无 ROS 依赖）+ `InterlockPolicy` + `RosAdapter`
- [ ] `arm_controller` 拆分 `MoveItFacade` + `TrajectoryExecutor` + `DriverBridge`；解决回调阻塞问题
- [ ] `grasp_perception` 拆分 `Detector`（YOLOv8/FoundationPose）+ `GraspPlanner`（候选生成 + 打分）+ `RosAdapter`

---

## 归档 / 弃用

- [ ] `defect_detector` 从 `system.launch.py` 移除；包源码保留在仓库直到正式清理
- [ ] `hikvision_driver` 默认不启用（`system.launch.py` 里 `use_hikvision:=false`），保留作为备用通用工业相机
- [ ] `demo操作指南.md`（原 8 点位拍照 demo）迁移为 `docs/legacy/inspection_demo.md`，新 demo 写在 `docs/GRASP_DEMO.md`

---

## 附录 A：巡检场景遗留任务（不再推进）

以下是原巡检方向下的任务，保留供溯源/对照：

- ~~`inspection_stations.yaml` 真机 8 点位标定~~（已在 demo 分支完成 8 点位，新方向用不到）
- ~~`defect_detector` 填充实际检测算法~~
- ~~深度补偿逻辑真机验证（±2cm 容差）~~
- ~~`task_coordinator` 订阅 `defect_detector` 结果 topic~~
- ~~端到端"导航+缺陷检测回看"链路~~

历史 commit：`eb114c9` / `dc886cf` / `4b374bf` / `d08a805`（巡检 8 点位 demo）
