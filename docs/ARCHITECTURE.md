# inspection-robot 系统架构文档

## 1. 项目概述

本项目当前方向为：**基于移动协作机械臂的大型工件视觉检测/巡检系统**（ROS2 Humble）。

硬件组成：

- 仙宫智能 AGV：负责大范围移动和站位导航。
- 大族 E05 / Elfin5 协作机械臂：负责相机末端姿态调整。
- Intel RealSense 深度相机：安装在机械臂末端，用于工作距测量与微调。
- 海康工业相机：安装在机械臂末端，用于高质量缺陷图像采集。
- Jetson Orin AGX 64G：运行 ROS2 驱动、控制、算法和状态机。

当前系统不以网页前端为主线，也不依赖 fake driver。历史存在的 `inspection_gateway` 和 `inspection_sim` 包保留源码，但不作为论文系统架构的必要组成。

技术方案：

```text
AGV 定点巡检
  -> 机械臂预设观测位
  -> RealSense 局部工作距测量与微调
  -> 海康工业相机触发拍照
  -> 缺陷检测
  -> ROS2 状态/结果输出
```

## 2. 系统分层

```text
┌─────────────────────────────────────────────────────────────┐
│                  ROS2 CLI / RViz / Launch                    │
│              任务启动、状态观察、实验数据记录                 │
└─────────────────────────────┬───────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    task_coordinator                          │
│         多站位巡检状态机 / 联锁门控 / 超时恢复                │
└───────────────┬──────────────────┬──────────────────────────┘
                │                  │
                ▼                  ▼
┌──────────────────────┐  ┌──────────────────────┐
│       驱动层          │  │       控制层          │
│ agv_driver           │  │ arm_controller        │
│ arm_driver           │  │ MoveIt2               │
│ realsense_driver     │  └──────────┬───────────┘
│ hikvision_driver     │             │
└──────────┬───────────┘             │
           │                         │
           ▼                         ▼
┌──────────────────────┐  ┌──────────────────────┐
│       算法层          │  │   inspection_interface │
│ defect_detector      │  │   msg / srv            │
└──────────────────────┘  └──────────────────────┘
```

## 3. 功能包职责

### 3.1 驱动层

| 包 | 职责 |
|---|---|
| `agv_driver` | 封装 AGV TCP API，接收站位目标并发布底盘状态 |
| `arm_driver` | 机械臂底层驱动，发布关节状态与设备状态 |
| `realsense_driver` | RealSense 适配层，提供 aligned depth、camera_info、TF |
| `hikvision_driver` | 海康工业相机驱动，提供 `image_raw`、`camera_info`、`trigger_capture` |

### 3.2 控制层

| 包 | 职责 |
|---|---|
| `arm_controller` | MoveIt2 运动控制，提供 `move_to_joints` 和 `move_to_pose` |

### 3.3 算法层

| 包 | 职责 |
|---|---|
| `defect_detector` | 工业图像缺陷检测，输出结构化缺陷结果 |

### 3.4 协调层

| 包 | 职责 |
|---|---|
| `task_coordinator` | 多站位巡检任务状态机，统一调度 AGV、机械臂、深度微调、拍照和检测 |

### 3.5 基础设施

| 包 | 职责 |
|---|---|
| `inspection_interface` | ROS2 msg/srv 定义 |
| `inspection_bringup` | launch 与配置入口 |
| `inspection_supervisor` | 系统健康监控，可选 |
| `inspection_gateway` | 历史 REST/WS 网关，当前非主线 |
| `inspection_sim` | 历史 fake driver 联调，当前非主线 |

## 4. 巡检状态机

```text
IDLE ──▶ MOVING_TO_STATION ──▶ ARM_PRESET ──▶ DEPTH_ADJUST ──▶ CAPTURING ──▶ 下一站 / COMPLETED
  ▲              │                  │               │               │
  │              ▼                  ▼               ▼               ▼
  │           PAUSED ◀────────── FAILED ◀──────────────────────────┘
  │              │
  └──────────────┘ STOP / RESUME
```

| 状态 | 动作 | 成功条件 |
|---|---|---|
| `IDLE` | 等待任务 | 接收到 start |
| `MOVING_TO_STATION` | 发布 AGV 目标站位 | AGV connected、arrived、stopped、error_code OK |
| `ARM_PRESET` | 调 `arm_control/move_to_joints` | 机械臂到预设关节角 |
| `DEPTH_ADJUST` | RealSense 测距并调 `move_to_pose` 微调 | 工作距误差小于容差 |
| `CAPTURING` | 触发海康拍照并调用 `defect_detector` | 得到检测结果或记录失败 |
| `COMPLETED` | 所有站位完成 | — |
| `FAILED` | 任一阶段超时或异常 | — |

## 5. ROS 接口

### 5.1 agv_driver

订阅：

- `goal_pose` (`geometry_msgs/msg/PoseStamped`)
- `goal_station` (`std_msgs/msg/String`，可选)
- `cmd_vel` (`geometry_msgs/msg/Twist`)

发布：

- `status` (`inspection_interface/msg/AgvStatus`)
- `current_pose` (`geometry_msgs/msg/PoseStamped`)
- `odom` (`nav_msgs/msg/Odometry`)

服务：

- `get_nav_map` (`inspection_interface/srv/GetNavMap`)：规划中，真机端尚需封装厂商地图 API。

### 5.2 arm_driver

订阅：

- `joint_cmd` (`sensor_msgs/msg/JointState`)

发布：

- `/joint_states` (`sensor_msgs/msg/JointState`)
- `status` (`inspection_interface/msg/ArmStatus`)

服务：

- `enable` / `disable` / `clear_fault` / `stop`

### 5.3 arm_controller

服务：

- `move_to_joints` (`inspection_interface/srv/MoveToJoints`)
- `move_to_pose` (`inspection_interface/srv/MoveToPose`)

发布：

- `motion_status`
- `trajectory_progress`

### 5.4 realsense_driver

常用发布：

- `/inspection/realsense/d435/color/image_raw`
- `/inspection/realsense/d435/aligned_depth_to_color/image_raw`
- `/inspection/realsense/d435/color/camera_info`
- `/inspection/realsense/d435/depth/color/points`（可选）

### 5.5 hikvision_driver

发布：

- `/inspection/hikvision/image_raw`
- `/inspection/hikvision/camera_info`

服务：

- `/inspection/hikvision/trigger_capture` (`std_srvs/srv/Trigger`)

### 5.6 defect_detector

订阅：

- `/inspection/hikvision/image_raw`

发布：

- `result` (`inspection_interface/msg/DefectInfo`)

服务：

- `detect_defect` (`std_srvs/srv/Trigger`)

### 5.7 task_coordinator

发布：

- `/inspection/state` (`inspection_interface/msg/SystemState`)
- `/inspection/agv/goal_pose`

订阅：

- `/inspection/agv/status`
- `/inspection/arm/status`
- `/inspection/realsense/d435/aligned_depth_to_color/image_raw`
- `/inspection/realsense/d435/color/camera_info`（增强微调需要）
- `/inspection/perception/result`

服务客户端：

- `/inspection/arm_control/move_to_joints`
- `/inspection/arm_control/move_to_pose`
- `/inspection/hikvision/trigger_capture`
- `/inspection/perception/detect_defect`

服务：

- `/inspection/start`
- `/inspection/stop`
- `/inspection/pause`
- `/inspection/resume`
- `/inspection/get_status`

配置：

- `stations_file`：指向 `inspection_stations.yaml`

## 6. TF 树

```text
map
 └─ base_link                 ← agv_driver 发布 map -> base_link
     └─ arm_base              ← 静态标定
         └─ elfin_link1..6
             └─ tool0
                 ├─ hikvision_frame    ← 海康工业相机外参
                 └─ camera_link        ← RealSense 外参
                     └─ camera_color_optical_frame
```

关键标定：

- `map -> base_link`：AGV 定位输出。
- `base_link -> arm_base`：机械臂安装位姿。
- `tool0 -> hikvision_frame`：工业相机外参。
- `tool0 -> camera_link`：RealSense 外参。

## 7. 工作距微调算法

### 7.1 基础版本

```text
1. 从 aligned depth 取目标 ROI。
2. 过滤无效深度。
3. 计算 ROI 深度中值 d_current。
4. 误差 e = d_current - d_target。
5. 若 |e| <= tolerance，进入拍照。
6. 否则 step = clamp(kp * e, -max_step, max_step)。
7. 将偏移转换到 arm_base / planning frame。
8. 当前 TCP 位姿叠加偏移，调用 move_to_pose。
```

注意：`MoveToPose.target_pose` 当前按绝对目标位姿解释，不能直接把 `position.z = delta` 当作相对偏移发送。

### 7.2 非平面增强版本

```text
1. ROI 深度反投影为局部点云。
2. 去除无效点和离群点。
3. PCA / RANSAC 拟合局部切平面。
4. 得到局部中心 c、法向 n、曲率指标。
5. 计算相机到局部切平面的距离。
6. 根据距离误差控制末端微调。
```

该方法只假设目标 ROI 内可近似为局部切平面，不假设整件工件是平面。

## 8. 缺陷检测算法

建议路线：

- 主线：YOLOv8-seg 缺陷实例分割。
- 备选：PatchCore / PaDiM 少样本异常检测。

输出应至少包含：

- defect_id
- defect_type
- confidence
- bbox 或 mask
- 缺陷面积
- 中心位置
- 标注图路径或文件 ID

当前 `defect_detector` 仍是骨架，实现缺陷检测是论文闭环的关键 P0。

## 9. Launch 约定

- 使用 YAML 配置，不在代码中硬编码。
- 图像类节点可使用 `ComposableNodeContainer` 降低拷贝。
- 使用 `TimerAction` 做必要的延迟启动，避免驱动与状态机竞争。
- 通过 launch 参数控制命名空间、配置文件、是否启动可选节点。

## 10. 验证方式

当前主线验证不依赖网页和 fake driver：

- 真机联调：AGV、机械臂、RealSense、海康相机、缺陷检测。
- 回放验证：用录制图像/深度数据验证微调和缺陷检测算法。
- ROS2 CLI/RViz：启动任务、观察状态、查看 TF、图像和检测结果。
- 实验文件：保存 CSV、原图、标注图，供论文统计。

