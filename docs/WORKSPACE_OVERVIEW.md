# 工作区总览（端到端约定）

本文档从系统全局视角约束“点位语义、导航地图、坐标系与标定、取图结果回显”等关键接口与实现边界，避免各模块 README 各写各的导致漂移。

## 1. 仓库分工（必须一致）

- `inspection-api/`
  - 只维护对外契约（gRPC proto / 可选 OpenAPI）。
  - “字段名 + 语义 + 进度推进规则”以 proto 为准。
- `inspection-hmi/`
  - Qt 上位机（Engineer/Operator），只对接 `inspection_gateway`（gRPC + 可选 HTTP 媒体直链）。
  - 不直接连 ROS2。
- `inspection-robot/`
  - 机器人端 ROS2 工作空间：驱动/控制/规划/检测/编排。
  - 部署 `inspection_gateway`（建议在 AGX 上），对外提供 gRPC 服务，对内调用 ROS2 节点。

整体链路：

`inspection-hmi -> inspection_gateway(gRPC) -> task_coordinator(ROS2) -> agv_driver/arm_controller/hikvision_driver/...`

## 2. 端到端流程（V1）

### 2.1 Engineer（配置与规划）

1. HMI 上传 CAD：`UploadCad` -> `model_id`
2. HMI 在 CAD 表面“点选”检测点位：
   - 每个点位包含：`SurfacePoint.position + SurfacePoint.normal`
   - “画笔”仅用于选点，不存在半径/强度
3. HMI 设置拍摄配置（整条任务统一）：
   - `CaptureConfig.focus_distance_m` 固定工作距离（工业相机对焦距离）
   - `CaptureConfig.max_tilt_from_normal_deg` 等角度约束（用于后续逆解/规划约束）
4. HMI 如有需要逐点调相机角度：
   - 默认 `ViewHint.view_direction = -SurfacePoint.normal`
   - 可调 `ViewHint.roll_deg`（绕光轴旋转，控制“图像上方向”）
5. HMI 下发点位：`SetInspectionTargets`
6. 触发规划：`PlanInspection` -> `plan_id + InspectionPath`
7. HMI 预览：
   - 导航路径折线：连接 `InspectionPath.waypoints[].agv_pose`（可仅用于显示下采样）
   - 机械臂目标：`arm_joint_goal`（或 `tcp_pose_goal/camera_pose` 若网关提供）

### 2.2 Operator（执行与结果）

1. `StartInspection(plan_id)`
2. HMI 订阅状态流：`SubscribeSystemState`（AGV/机械臂/联锁/进度）
3. HMI 订阅事件流：`SubscribeInspectionEvents`（抓拍/缺陷/告警）
4. 原图下载：
   - 优先用事件里的 `thumbnail_jpeg` 做实时预览
   - 需要原图时 `DownloadMedia(media_id)`（或 `MediaRef.url` 直链）
5. 任务结束后按点位/任务回看：`ListCaptures`

## 3. “路径折线”能否获取？

可以。V1 的“路径折线”定义为：**规划输出的站位点（waypoints）按顺序连线**，数据来自：

- `PlanInspectionResponse.path.waypoints[].agv_pose`
- 或断点恢复 `GetPlanResponse.path.waypoints[].agv_pose`

注意：这不是底盘内部规划器的细粒度轨迹（如果未来需要“真实路径采样点”，再单独扩展接口或对接底盘能力）。

## 4. 导航地图 GetNavMap（地图在 AGV 上）

结论：AGV 侧存在地图与查询/下载接口，网关可以自动获取并缓存后返回给 HMI。

推荐实现策略（对齐“厂商协议仅在 driver 内使用”的分层原则）：

1. `agv_driver` 封装厂商 TCP API：
   - `1300 robot_status_map_req`：获取当前载入地图名与 md5、保存的地图列表
   - `4011 robot_config_downloadmap_req`：按 map_name 下载 `.smap`（JSON 文本）
   - 可选 `1513 robot_status_costmap_req`：取代价地图（Message_Grid），适合生成栅格底图
2. `inspection_gateway` 通过 ROS2 service 向 `agv_driver` 请求“地图元信息 + 底图”，并做缓存（key 建议用 `map_name + md5`）
   - ROS2 service（建议）：`/inspection/agv/get_nav_map`（`inspection_interface/srv/GetNavMap`）
3. 网关对外实现 `GetNavMap`，返回：
   - `resolution_m_per_pixel / origin / width / height / image(ImageRef)`

### 4.1 地图坐标与像素坐标

统一约定：**AGV 位姿、规划点位、机械臂基座/末端位姿都使用同一个 `map` 坐标系（米/弧度）**。

`GetNavMap` 返回的 `origin` 是像素 `(u=0,v=0)` 在 `map` 坐标系里的位置；HMI 侧投影公式（同 proto 注释）：

- `u = (x - origin.x) / resolution`
- `v = (origin.y - y) / resolution`

其中 `(x,y)` 是任意 `Pose2D` 的 `map` 坐标。

## 5. 坐标系与标定（决定“能否对齐”）

### 5.1 最小 TF 链

要实现“导航 + 机械臂位姿 + 相机取图姿态”在同一视图对齐，机器人端必须具备最小 TF 链：

- `map -> base_link`：由 `agv_driver` 发布（或由网关转发布）
- `base_link -> arm_base`：机械臂安装位姿（静态标定，必须配置）
- `arm_base -> tcp`：由机械臂 URDF/关节状态正解得到
- `tcp -> camera`：相机外参（静态标定，必须配置）

### 5.2 相机角度能否传给机械臂？

能。HMI 通过 `InspectionTarget.view` 下发“相机拍摄角度”（方向 + roll），网关在规划/执行阶段用相机外参把“相机目标位姿”转换成“TCP 目标位姿”，再做逆解得到 `arm_joint_goal`。

典型计算（概念层面）：

- `camera_pos = surface_pos - view_direction * focus_distance_m`
- `T_map_tcp_goal = T_map_camera_goal * inverse(T_tcp_camera)`

其中 `T_tcp_camera` 就是“相机相对末端”的外参配置。

### 5.3 外参在哪里配？

建议唯一来源：URDF（固定关节）或静态 TF（带参数文件），禁止散落在多个节点里硬编码。

当前仓库里存在示例静态 TF（注意 frame 名称需要与 URDF 一致）：

- `src/inspection_bringup/launch/drivers.launch.py`

如果 URDF 的末端 link 不是 `tool0`，需要统一改成真实末端（例如 `elfin_end_link`）或在 URDF 增加 `tool0` 别名 link。

