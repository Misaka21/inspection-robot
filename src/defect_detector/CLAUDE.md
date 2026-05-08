# defect_detector/CLAUDE.md

本包是当前巡检方向的**主算法包**，负责从海康工业相机图像中检测工件表面缺陷，并发布结构化结果。

## 1. 包职责与边界

负责：

- 缓存 `/inspection/hikvision/image_raw` 最新图像。
- 在 `detect_defect` service 被触发时执行缺陷检测。
- 发布 `inspection_interface/msg/DefectInfo`。
- 保存原图/标注图/实验统计数据（可放到独立工具类或后续 `capture_manager`）。

不负责：

- 相机 SDK 调用。
- AGV / 机械臂任务编排。
- 网页、REST/WS 推送。

## 2. Public ROS API

默认命名空间：`/inspection/perception`

订阅：

- `/inspection/hikvision/image_raw` (`sensor_msgs/msg/Image`)

发布：

- `result` (`inspection_interface/msg/DefectInfo`)

服务：

- `detect_defect` (`std_srvs/srv/Trigger`)

## 3. 推荐内部架构

建议拆为：

1. `DefectModel`
   - YOLOv8-seg / PatchCore / PaDiM / ONNXRuntime / TensorRT 的统一接口。
2. `Preprocess`
   - resize、normalize、ROI 裁剪、色彩空间转换。
3. `Postprocess`
   - NMS、mask 阈值、连通域分析、面积/中心计算。
4. `RosAdapter`
   - 缓存最新图像。
   - service 触发时复制图像并调用算法。
   - 发布结果与保存实验文件。

约束：

- 图像回调只缓存，不要直接跑推理。
- `detect_defect` 应使用最近一次海康触发拍照后的新图像。
- 推理结果需要能用于论文统计，不只返回 success/fail。

## 4. 算法路线

主路线：

- YOLOv8-seg：适合有缺陷标注数据的场景，输出类别、bbox、mask、置信度。

备选路线：

- PatchCore / PaDiM：适合缺陷样本少、正常样本较多的场景，输出异常热力图。

后处理建议输出：

- defect_type
- confidence
- bbox / mask
- area
- center
- annotated_image_path

## 5. 文档与 TODO 维护

修改检测接口、输出字段或算法路线时，必须同步更新：

- `TODO.md`
- `docs/ARCHITECTURE.md`
- `docs/IMPLEMENTATION_STATUS.md`
- `docs/THESIS_PROJECT_SUMMARY.md`

