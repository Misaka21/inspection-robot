# inspection_interface/CLAUDE.md

本包是机器人端 ROS2 内部契约层，当前主线服务于**巡检/视觉检测系统**。

## 1. 边界与优先级

优先级：

1. `inspection_interface`：跨包 ROS2 msg/srv 契约。
2. 各 driver/controller/algo 包内部实现。
3. 厂商协议，仅允许出现在对应 driver 内。

当前不以 `inspection_gateway/api/models.py` 为最高优先级，因为网页/REST/WS 不是当前论文主线。

## 2. 当前主线接口

状态：

- `AgvStatus.msg`
- `ArmStatus.msg`
- `SystemState.msg`
- `DefectInfo.msg`

运动控制：

- `MoveToJoints.srv`
- `MoveToPose.srv`

任务控制：

- `StartInspection.srv`
- `StopInspection.srv`
- `PauseInspection.srv`
- `ResumeInspection.srv`
- `GetInspectionStatus.srv`

可选/历史：

- `GetNavMap.srv`
- `DioStatus.msg`
- `SetDioOutput.srv`
- `InspectionPath` / `InspectionTask` / `InspectionWaypoint` 等历史消息

## 3. 待评估扩展

巡检论文闭环可能需要增强 `DefectInfo.msg`，例如：

- bbox
- mask 或 mask 文件路径
- defect_area
- center_pixel
- annotated_image_path
- station_name / task_id

如果新增字段，应优先保持向后兼容；若破坏现有字段语义，提交信息必须标记 `BREAKING CHANGE`。

## 4. 演进规则

- msg/srv 字段以 `.msg/.srv` 文件为准，文档不要复制太多字段表。
- 不为当前不用的抓取方向新增 `GraspPose`、`StartGrasp`、`GripperStatus` 等接口。
- 修改字段语义必须同步更新：
  - `docs/ARCHITECTURE.md`
  - `docs/IMPLEMENTATION_STATUS.md`
  - `docs/WORKSPACE_OVERVIEW.md`
  - `TODO.md`

