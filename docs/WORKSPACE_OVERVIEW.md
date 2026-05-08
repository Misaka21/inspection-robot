# 工作区总览（巡检方向）

本文档从系统全局视角约束当前论文主线：**移动协作机械臂大型工件视觉检测/巡检**。

当前默认：

- 不依赖网页前端。
- 不依赖 fake driver。
- 通过 ROS2 launch、RViz、ros2 CLI、实验脚本和真机/回放数据完成演示与论文验证。

## 1. 仓库分工

`inspection-robot/` 是机器人端 ROS2 工作空间，主线包含：

- AGV 底盘驱动
- 机械臂驱动与 MoveIt2 控制
- RealSense 深度相机
- 海康工业相机
- 缺陷检测算法
- 巡检任务状态机
- launch/config 与实验记录

历史/可选模块：

- `inspection_gateway`：REST/WS 网关，当前无网页主线，不作为论文必需。
- `inspection_sim`：fake driver 仿真，当前不作为论文主线。

## 2. 端到端巡检流程

```text
1. 通过 ros2 launch 启动驱动和系统
2. task_coordinator 加载 inspection_stations.yaml
3. 通过 ros2 service 启动巡检
4. AGV 到达当前站位
5. 机械臂运动到该站位预设观测关节角
6. RealSense 采集深度图，微调相机工作距
7. 海康工业相机触发拍照
8. defect_detector 对图像进行缺陷检测
9. 结果通过 ROS2 topic / 文件落盘输出
10. 进入下一站位，直到完成
```

## 3. 站位配置

站位配置文件：`src/inspection_bringup/config/inspection_stations.yaml`

```yaml
stations:
  - name: "station_1"
    agv_pose:
      x: 1.0
      y: 2.0
      z: 0.0
      yaw: 0.5
    arm_joints: [0.0, -0.5, 1.2, 0.0, 0.8, 0.0]
    target_distance: 0.30
    distance_tolerance: 0.02
    adjust_axis: "z"
```

含义：

- `agv_pose`：AGV 在地图坐标系下的站位。
- `arm_joints`：机械臂预设观测姿态。
- `target_distance`：相机到工件局部表面的目标工作距。
- `distance_tolerance`：允许误差。
- `adjust_axis`：基础微调方向；增强版应使用 TF 和局部表面法向。

## 4. 坐标系与标定

最小 TF 链：

```text
map
 └─ base_link
     └─ arm_base
         └─ tool0
             ├─ hikvision_frame
             └─ camera_link
                 └─ camera_color_optical_frame
```

必须标定：

- `base_link -> arm_base`：机械臂安装位姿。
- `tool0 -> hikvision_frame`：工业相机外参。
- `tool0 -> camera_link`：RealSense 外参。

巡检方向的精度重点不是抓取位姿，而是：

- AGV 到站位后，目标区域能进入机械臂观测范围。
- 机械臂预设位姿能让相机看到检测区域。
- RealSense 测距与海康拍照目标区域在机械结构上能对齐。
- 工作距微调后，海康图像满足检测清晰度和尺度一致性。

## 5. 工作距微调数据流

```text
realsense_driver
  -> aligned_depth_to_color/image_raw
  -> camera_info
  -> task_coordinator / DepthAdjustCore
  -> 估计 ROI 当前工作距
  -> 生成 TCP 绝对目标位姿
  -> arm_controller/move_to_pose
```

基础版：

- ROI 深度中值估计距离。

增强版：

- ROI 深度反投影成局部点云。
- PCA / RANSAC 估计局部切平面。
- 用局部切平面距离计算工作距误差。

## 6. 缺陷检测数据流

```text
task_coordinator
  -> hikvision/trigger_capture
  -> hikvision/image_raw
  -> defect_detector 缓存最新图像
  -> task_coordinator 调 detect_defect
  -> defect_detector/result
  -> 文件落盘 / CSV / 论文统计
```

`defect_detector` 不应在图像回调中直接做耗时推理。建议：

- 图像回调只缓存最新图像。
- service 触发时复制缓存并推理。
- 推理结束后发布 `DefectInfo` 和保存标注图。

## 7. 结果展示

当前不依赖网页，建议用以下方式展示：

- RViz：TF、机械臂、相机坐标系、AGV 位置。
- `rqt_image_view`：海康图像与 RealSense 图像。
- `ros2 topic echo /inspection/state`：状态机阶段。
- `ros2 topic echo /inspection/perception/result`：缺陷结果。
- 实验目录：保存原图、标注图、CSV。

## 8. 非主线模块说明

### inspection_gateway

历史 FastAPI 网关，若后续恢复网页可再整理；当前论文不以它作为系统必要链路。

### inspection_sim

历史 fake driver 联调包；当前论文不以 fake driver 作为系统实现依据。可保留代码，但文档和实验不要把它作为核心能力。

