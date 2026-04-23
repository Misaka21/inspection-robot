# defect_detector/CLAUDE.md

> ⚠️ **状态：巡检场景遗留包，抓取方向不再使用。**
>
> 本包为原"大型工件视觉检测（巡检）"方向实现的工程骨架。课题已调整为**复合机器人抓取**，新的算法模块是 `grasp_perception`（RGBD → 抓取位姿）。本包保留源码在仓库中但从 `system.launch.py` 中移除（见 `TODO.md` 归档节）。
>
> 如需再启用，或把 defect 能力作为抓取成功复核（"抓到的物体看起来对不对"）的补充，请先在 `TODO.md` 中登记新用例，再讨论接口形态。

## 1. 包职责与边界（原设计）

负责（原）：
- 从工业相机图像做缺陷检测
- 发布结构化缺陷结果（当前为工程骨架）

不负责：
- 相机驱动（`hikvision_driver`）
- 媒体存储与对外下载（网关/媒体模块）

## 2. Public ROS API（当前骨架，不再推进）

默认命名空间：`/inspection/perception`

订阅：
- `/inspection/hikvision/image_raw` (`sensor_msgs/msg/Image`)

发布：
- `result` (`inspection_interface/msg/DefectInfo`)

服务：
- `detect_defect` (`std_srvs/srv/Trigger`)

## 3. 在抓取方向中的定位

- `system.launch.py` 不再启动本节点
- `task_coordinator` 不再调用 `detect_defect`
- `inspection_interface/msg/DefectInfo` 保留定义（不破坏原有消息库）；未来可考虑清理

## 4. 如果要重新启用（例如做"抓后复核"）

建议路径：
- 在 `grasp_perception` 中扩展一个 `verify_grasp` 接口（已抓到的物体是否符合预期），而不是复活 `defect_detector`
- 真的要用本包做 defect 视觉检查时，订阅源改为 RealSense RGB（而非 hikvision），并把结果挂到 `grasp_event` 而非单独 topic

## 5. 文档与 TODO 维护（必须）

- 本包不再维护新功能；如有使用或改造，先更新 `TODO.md` 再动手
