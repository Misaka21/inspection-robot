# 实现状态与代码规划（巡检方向）

本文档按当前论文方向“基于移动协作机械臂的大型工件视觉检测/巡检系统”梳理代码现状和缺口。

> 当前不以网页前端为主线，不依赖 fake driver。`inspection_gateway` 和 `inspection_sim` 保留源码，但不作为当前论文系统闭环的必要模块。

## 1. 快速结论

### 已有基础

- `agv_driver`：已有底盘驱动、状态发布、目标位姿下发等基础能力。
- `arm_driver`：已有机械臂底层驱动、关节状态和基础服务。
- `arm_controller`：已有 MoveIt2 `move_to_joints` / `move_to_pose` 服务。
- `realsense_driver`：已有 RealSense 启动配置，支持 aligned depth。
- `hikvision_driver`：已有工业相机驱动、`image_raw` 发布和 `trigger_capture` 服务。
- `task_coordinator`：已有巡检状态机骨架，支持 YAML 站位、AGV 门控、机械臂预设、深度中值微调和缺陷检测触发。
- `inspection_interface`：已有巡检相关 msg/srv，例如 `SystemState`、`DefectInfo`、`MoveToPose`、`MoveToJoints`。
- `inspection_bringup`：已有 drivers/system launch 和站位配置。

### 核心缺口

- `defect_detector` 仍是骨架，没有真实缺陷检测算法。
- `task_coordinator` 的 `CAPTURING` 阶段尚未形成“触发海康拍照 -> 等待新图 -> 缺陷检测 -> 收集结果”的闭环。
- 深度微调当前只是中心 ROI 中值，且位姿修正语义需要修正：`MoveToPose` 接收绝对位姿，不能直接传相对 `delta`。
- 非平面局部点云微调算法尚未实现。
- 真机 `agv_driver/get_nav_map` 地图服务尚未实现；当前论文如不展示网页地图，可不作为 P0。
- 网页、REST/WS、fake driver 相关功能不作为当前主线。

## 2. 模块状态表

| 模块 | 当前状态 | 为论文闭环还需补齐 |
|---|---|---|
| `agv_driver` | 基础驱动可用 | 真机站位验证；可选补 `get_nav_map` |
| `arm_driver` | 基础驱动可用 | 真机关节状态与到位判定验证 |
| `arm_controller` | MoveIt2 服务可用 | 暴露/记录当前 TCP 位姿；支持微调绝对目标生成 |
| `realsense_driver` | aligned depth 配置可用 | 验证 depth 与 ROI 对齐；订阅 camera_info |
| `hikvision_driver` | 触发拍照服务可用 | 与 task_coordinator 串联，确认新图同步机制 |
| `defect_detector` | 骨架 | 实现 YOLOv8-seg / PatchCore 等真实算法 |
| `task_coordinator` | 巡检状态机骨架 | 修正微调、串联拍照检测、订阅检测结果 |
| `inspection_interface` | 基础 msg/srv 已有 | 视缺陷输出需求扩展 bbox/mask/面积字段 |
| `inspection_bringup` | launch/config 已有 | 与巡检主线参数对齐，清理抓取残留 |
| `inspection_gateway` | 历史可选 | 当前非主线，不作为论文验收 |
| `inspection_sim` | 历史可选 | 当前非主线，不作为论文验收 |

## 3. 当前执行链路与缺口

### 3.1 目标链路

```text
ros2 service call /inspection/start
  -> task_coordinator 加载 inspection_stations.yaml
  -> MOVING_TO_STATION: 发布 /inspection/agv/goal_pose
  -> ARM_PRESET: 调 /inspection/arm_control/move_to_joints
  -> DEPTH_ADJUST: 读取 RealSense aligned depth 并微调 TCP
  -> CAPTURING: 调 /inspection/hikvision/trigger_capture
  -> defect_detector: 对最新海康图像推理
  -> 发布 /inspection/perception/result
  -> task_coordinator 记录结果并进入下一站
```

### 3.2 当前代码差距

- `DEPTH_ADJUST`：
  - 已有中心 ROI 中值测距。
  - 需要修正相对/绝对位姿语义。
  - 需要扩展到 `camera_info` + 局部点云 + PCA/RANSAC。

- `CAPTURING`：
  - 当前主要触发 `detect_defect`。
  - 需要先触发 `hikvision/trigger_capture`，并保证检测使用的是该站位新图。

- `defect_detector`：
  - 当前收到图像后发布占位 `none` 结果。
  - 需要实现真实模型加载、推理和后处理。

## 4. 论文实验需要补的数据

| 实验 | 需要产物 |
|---|---|
| 工作距微调 | 微调前后距离误差 CSV、收敛次数、失败率 |
| 非平面适应 | 不同倾角/曲率 ROI 的距离估计误差 |
| 缺陷检测 | 数据集、标注、Precision、Recall、mAP/F1、推理耗时 |
| 多站位巡检 | 单站耗时、整轮耗时、连续运行成功率 |
| 系统演示 | RViz 截图、ros2 topic/service 日志、原图与标注图 |

## 5. 建议落地顺序

1. 文档与 launch 口径全部回到巡检方向。
2. 修正 `DEPTH_ADJUST` 的位姿控制语义。
3. 补 `CameraInfo` 订阅与局部点云工作距估计。
4. 串联 `trigger_capture -> detect_defect -> result`。
5. 实现 `defect_detector` 真实检测算法。
6. 保存实验数据与标注图。
7. 真机多站位跑通并统计论文指标。

