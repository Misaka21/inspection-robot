# 工作区总览（端到端约定）

本文档从系统全局视角约束"抓取任务、导航地图、坐标系与标定、结果回显"等关键接口与实现边界，避免各模块 README 各写各的导致漂移。

> **课题方向**：复合移动机器人抓取（非巡检）。仓库名 `inspection-robot` 与 `inspection_*` 包名是历史遗留。见 `README.md` / `CLAUDE.md`。

实现落地与缺口清单见：`docs/IMPLEMENTATION_STATUS.md`。

## 1. 仓库分工（必须一致）

- `inspection-site/`（独立仓库）
  - React 前端（Operator 界面为主），对接 `inspection_gateway`（REST API + WebSocket）
  - 不直接连 ROS2
  - 构建产物部署到 `inspection_gateway/frontend/dist/`，由 FastAPI 托管
- `inspection-robot/`（本仓库）
  - 机器人端 ROS2 工作空间：驱动 / 控制 / 感知 / 夹爪 / 编排
  - 部署 `inspection_gateway`（运行在 Jetson Orin AGX），对外提供 REST/WS 服务，对内调用 ROS2 节点
  - 网关代码落点：`src/inspection_gateway/`（ROS2 包，FastAPI server + ROS2 bridge + store）
  - API 契约的唯一事实来源：`inspection_gateway/api/models.py`（Pydantic v2）

> 历史说明：`inspection-api`（gRPC proto）和 `inspection-hmi`（Qt gRPC 上位机）已归档。

整体链路：

```
inspection-site (React) ← REST/WS → inspection_gateway (FastAPI)
    → task_coordinator (ROS2)
        → agv_driver / arm_controller / grasp_perception / gripper_driver / ...
```

## 2. 端到端流程（抓取 V1）

### 2.1 Operator（执行与结果）

1. 选择或录入任务参数：
   - 目标物体类别 `object_class`（可选；不填则感知使用"任意可抓取物体"策略）
   - 抓取点位 / 放置点位（从 yaml 模板选，或自定义）
2. `POST /api/v1/tasks` 触发抓取（`StartGrasp`）
3. 前端订阅：`WS /ws`
   - `system_state` 实时状态（当前阶段、AGV/Arm 状态）
   - `grasp_event` 结构化事件（PERCEIVED / GRASPED / PLACED / FAILED…）【待实现】
4. 观察图/抓取瞬间图：
   - 优先用事件里的 `thumbnail_jpeg` 做实时预览
   - 需要原图时 `GET /api/v1/media/{media_id}`
5. 任务结束后按任务回看：`GET /api/v1/tasks/{id}/captures`【待实现】

### 2.2 Engineer（配置）

当前抓取 V1**不需要 CAD 表面选点**这种流程（那是巡检/检测才需要）。Engineer 主要工作是：

- 新工件类别 → 采集 YOLOv8 数据集 → 训练 → 权重更新
- 每类工件的默认抓取姿态写到 `grasp_perception/config/class_grasp_poses.yaml`
- AGV 抓取/放置点位写到 `task_coordinator/config/grasp_tasks.yaml`

> 可选升级：走 FoundationPose 6D 位姿路径时，仍需要 CAD → 保留 `/cad/upload` 接口即可；但不再有"在 CAD 表面画笔选点"那一套。

## 3. 抓取任务配置

`task_coordinator/config/grasp_tasks.yaml` 示例：

```yaml
tasks:
  demo_pick_screw:
    object_class: "screw_m8"
    pick:
      use_agv: true
      agv_station: "PICK_01"        # 或 agv_pose: {x, y, yaw}
      arm_observe_joints: [0.0, -0.5, 1.2, 0.0, 0.8, 0.0]
      approach_offset_m: 0.10       # pre-grasp 距抓取点高度
      velocity_scaling: 0.2
    place:
      use_agv: true
      agv_station: "PLACE_01"
      arm_place_joints: [0.0, 0.3, 0.8, 0.0, 1.1, 0.0]
      release_offset_m: 0.05        # 松开前抬起距离
    recovery:
      max_perceive_retries: 3
      max_grasp_retries: 2
```

## 4. 导航地图 GetNavMap（地图在 AGV 上）

AGV 侧存在地图与查询/下载接口，网关自动获取并缓存后返回给前端。推荐实现策略（对齐"厂商协议仅在 driver 内使用"的分层原则）：

1. `agv_driver` 封装厂商 TCP API（命令号/协议细节仅在 driver 内部），对外暴露 ROS2 service
2. `inspection_gateway` 通过 ROS2 service 向 `agv_driver` 请求"地图元信息 + 底图"，并做缓存（key 建议用 `map_name + md5`）
   - ROS2 service：`/inspection/agv/get_nav_map`（`inspection_interface/srv/GetNavMap`）
3. 网关对外实现 `GET /api/v1/nav/map`，返回：
   - `resolution_m_per_pixel / origin / width / height / image(ImageRef)`

### 4.1 地图坐标与像素坐标

统一约定：**AGV 位姿、抓取/放置站位、机械臂基座都使用同一个 `map` 坐标系（米/弧度）**。

`GetNavMap` 返回的 `origin` 是像素 `(u=0,v=0)` 在 `map` 坐标系里的位置；前端投影公式：

- `u = (x - origin.x) / resolution`
- `v = (origin.y - y) / resolution`

其中 `(x,y)` 是任意 `Pose2D` 的 `map` 坐标。

## 5. 坐标系与标定（决定"能否抓中"）

### 5.1 最小 TF 链

```
map
 └─ base_link (AGV)                    ← agv_driver 发布 map -> base_link
      └─ arm_base                        ← 静态标定（yaml，必须补齐）
           └─ elfin_link1..6              ← URDF + joint_states 正解
                └─ tool0
                     ├─ gripper_tip       ← 静态 TF（气动夹爪末端 offset）
                     └─ camera_link       ← 手眼标定（tool0 → camera_link，yaml）
                          └─ camera_color_optical_frame ← realsense2_camera 自发
```

### 5.2 手眼标定（抓取精度上限的关键）

- 工具：`easy_handeye2` + ChArUco 板
- 标定流程：把 ChArUco 板固定在工作台；机械臂带相机拍不同姿态 N 次；求解 `T_tool0_camera`
- 结果存 `inspection_bringup/config/handeye_calib.yaml`，系统启动时通过 `static_transform_publisher` 广播 `tool0 → camera_link`
- 验证：RViz 下叠加 PointCloud 和 URDF，工作台的点云与工作台实物位置对齐（误差 < 5mm 为可接受）

### 5.3 抓取位姿变换链

感知输出位姿在 `camera_color_optical_frame`（`grasp_perception` 写入 response.frame_id）；`task_coordinator` 执行时：

```
T_armbase_grasp = T_armbase_base * T_base_tool0 * T_tool0_camera * T_camera_grasp
```

其中：
- `T_armbase_base`：`arm_base → base_link` 的逆（静态 TF）
- `T_base_tool0`：URDF + 当前关节角正解
- `T_tool0_camera`：手眼标定结果
- `T_camera_grasp`：感知模块输出

实现时直接用 `tf2` buffer 查一次 `camera_color_optical_frame → arm_base` 即可，不用手算。

## 6. 结果回显与事件流

### 6.1 拍照点

建议的事件 / 抓拍时机（都走 `capture_manager`，落盘成 `media_id`）：

| 时机 | 事件类型 | 内容 |
|------|---------|------|
| 到观察位后 CAPTURE_RGBD | `OBSERVED` | 观察图（RGB） |
| PERCEIVE 成功 | `PERCEIVED` | 观察图 + 候选抓取位姿标注图 |
| GRASP 完成 LIFT 后 | `GRASPED` | 抓起后的观察图（验证是否真夹到） |
| PLACE 完成 | `PLACED` | 放置点位观察图 |
| 任何失败 | `FAILED` | 失败上下文 |

### 6.2 WebSocket 协议

- `system_state`：实时状态（`TaskStatus`，含阶段、AGV/Arm/Gripper 状态、进度）
- `grasp_event`：结构化事件（`GraspEvent`，含 `type`、`media_id`、时间戳、上下文）

两个类型通过 `WsEnvelope.type` 区分。

## 7. 旧巡检概念的下线清单

原巡检流程里的以下概念在抓取场景中**不再使用**（保留在 models.py 中仅为溯源，前端已无对应 UI）：

- `POST /cad/upload` + `SurfacePoint/InspectionTarget`（CAD 表面选点）
- `InspectionPath` / `InspectionPoint`（路径/点位预览，改为 `grasp_tasks.yaml` 配置驱动）
- `defect_detector` 调用与 `DefectInfo`
- `ViewHint.focus_distance_m` 等拍摄配置（抓取场景不需要手动定焦距）

> 实现上优先路径：**先留字段不用**，等抓取方向稳定后再从 models.py 和前端删减。
