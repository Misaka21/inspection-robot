# TODO（inspection-robot，巡检系统）

本文件维护“还要做什么”的单一事实来源。当前课题方向为：**基于移动协作机械臂的大型工件视觉检测/巡检系统**。

> 说明：
> - 不再推进复合机器人抓取方向；`grasp_perception` / `gripper_driver` / 抓取 pipeline 相关任务移出主线。
> - 当前论文与演示不依赖网页前端；`inspection_gateway` 保留为历史/可选模块。
> - 当前论文与演示不依赖 fake driver；`inspection_sim` 保留为历史/可选模块，不作为主线验收依据。

相关参考：
- 主说明：`README.md`
- 系统架构：`docs/ARCHITECTURE.md`
- 落地缺口：`docs/IMPLEMENTATION_STATUS.md`
- 论文说明：`docs/THESIS_PROJECT_SUMMARY.md`

---

## P0（必须，先让巡检闭环真实成立）

### 任务编排
- [ ] 确认 `task_coordinator` 主状态机固定为巡检流程：`MOVING_TO_STATION -> ARM_PRESET -> DEPTH_ADJUST -> CAPTURING`
- [ ] `task_coordinator` 在 `CAPTURING` 阶段先调用 `hikvision/trigger_capture`，等待新图像后再触发 `defect_detector/detect_defect`
- [ ] `task_coordinator` 订阅 `defect_detector/result`，保存每个站位的缺陷结果，不只看 Trigger 返回值
- [ ] `inspection_stations.yaml` 真机标定：填入实际 AGV 站位坐标、机械臂预设关节角、目标工作距和容差

### 工作距微调
- [ ] 修正当前深度微调的位姿语义：不要把 `MoveToPose.target_pose` 当相对位移；应读取当前 TCP 位姿，叠加偏移后发送绝对目标位姿
- [ ] 订阅 `camera_info`，支持 ROI 深度反投影为局部点云
- [ ] 实现 ROI 有效深度滤波：无效值过滤、深度范围约束、中值/分位数滤波
- [ ] 实现非平面局部表面估计：PCA 或 RANSAC 拟合局部切平面，输出中心点、法向和曲率指标
- [ ] 增加微调参数：`roi_center` / `roi_size` / `kp` / `max_step` / `min_valid_points` / `curvature_threshold`
- [ ] 真机验证深度微调：微调后工作距误差达到论文目标（建议 ±2 cm）

### 缺陷检测
- [ ] `defect_detector` 从骨架改为真实算法：缓存最新图像，service 触发时推理，不在图像回调里阻塞推理
- [ ] 选择并实现主算法：
  - YOLOv8-seg：有标注缺陷数据时使用
  - PatchCore / PaDiM：缺陷样本少时作为异常检测路线
- [ ] 完成图像预处理、推理、后处理、NMS/阈值分割
- [ ] 扩展或复用 `DefectInfo.msg`，输出缺陷类别、置信度、bbox/mask、面积、中心位置
- [ ] 输出标注图，至少能落盘到实验目录供论文使用

### 相机与标定
- [ ] 标定并固定 `base_link -> arm_base`
- [ ] 标定并固定 `tool0 -> hikvision_frame`
- [ ] 标定并固定 `tool0 -> camera_link`（RealSense）
- [ ] 确认海康相机触发模式、曝光、增益、标定参数在 launch/config 中可复现
- [ ] 确认 RealSense aligned depth 与目标 ROI 对齐可用

---

## P1（论文实验与结果链路）

- [ ] 新增或整理实验脚本：保存每站位原图、深度 ROI、微调前后距离、缺陷检测结果和耗时 CSV
- [ ] 设计深度微调实验：平面、倾斜面、曲面/局部非平面各自统计误差
- [ ] 设计缺陷检测实验：Precision、Recall、mAP、F1、推理耗时
- [ ] 设计多站位巡检实验：单站耗时、整轮耗时、连续运行成功率
- [ ] 整理 RViz/ros2 CLI 演示流程，不依赖网页前端
- [ ] `inspection_supervisor` 订阅 `/inspection/agv/status`、`/inspection/arm/status`、`/inspection/state`，输出健康状态

---

## P2（工程化）

- [ ] `task_coordinator` 拆分 `CoordinatorCore`（无 ROS 依赖）+ `InterlockPolicy` + `RosAdapter`
- [ ] `arm_controller` 拆分 `MoveItFacade` + `TrajectoryExecutor` + `DriverBridge`
- [ ] `hikvision_driver` 拆分 `HkSdkSession` + `GrabWorker` + `MonitorWorker` + `RosAdapter`
- [ ] `defect_detector` 拆分 `DefectModel` + `Preprocess` + `Postprocess` + `RosAdapter`
- [ ] 为关键逻辑补单测：站位 YAML 解析、深度 ROI 统计、局部点云拟合、状态机推进
- [ ] Bringup 参数体系收敛：topic/service 名称、namespace、相机参数、微调参数全部从 YAML / launch 控制
- [ ] 统一历史代码中的私有成员命名为 `_prefix`

---

## 归档 / 非主线

- [ ] `inspection_gateway`：保留源码，不作为当前论文和演示主线；若后续恢复网页再单独整理
- [ ] `inspection_sim`：保留源码，不作为当前论文和演示主线；当前不依赖 fake driver
- [x] `docs/GRASP_DEMO.md`：标记为历史抓取方向文档
- [x] 清理主文档中的抓取方向残留表述
