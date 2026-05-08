# hikvision_driver/CLAUDE.md

本包是当前巡检方向的**主图像采集驱动**，负责通过海康工业相机采集缺陷检测图像。

## 1. 包职责与边界

负责：

- 通过海康 MV SDK 取图。
- 发布 `image_raw` + `camera_info`。
- 提供 `trigger_capture` 触发服务。
- 支持触发模式和基础相机参数配置。

不负责：

- 缺陷检测算法。
- 巡检任务状态机。
- 文件落盘和实验 CSV 统计。

## 2. Public ROS API

默认命名空间：`/inspection/hikvision`

发布：

- `image_raw`（image_transport）
- `camera_info`

服务：

- `trigger_capture` (`std_srvs/srv/Trigger`)

常用参数：

- `sn` / `device_index`
- `exposure_time`
- `gain`
- `frame_rate`
- `use_trigger_mode`
- `camera_info_url`

## 3. 推荐内部架构

建议按职责拆分：

1. `HkSdkSession`
   - SDK 句柄 create/open/close/destroy。
   - 参数设置。
2. `GrabWorker`
   - blocking grab、像素格式转换、发布图像。
3. `MonitorWorker`
   - 掉线重连、失败计数、backoff。
4. `RosAdapter`
   - 参数声明、pub/service 创建、trigger flag。

约束：

- service 回调不要直接执行长时间阻塞 SDK 操作。
- SDK 句柄生命周期集中管理。
- `trigger_capture` 成功后应能让 `task_coordinator` 判断是否收到新图像。

## 4. 与巡检状态机的关系

`CAPTURING` 阶段目标顺序：

```text
task_coordinator -> hikvision/trigger_capture
hikvision_driver -> image_raw
defect_detector 缓存该图像
task_coordinator -> perception/detect_defect
```

driver 只负责可靠发布图像，不负责检测和结果管理。

## 5. 文档与 TODO 维护

修改 public ROS API、参数名或触发行为时，必须同步更新：

- `TODO.md`
- `docs/ARCHITECTURE.md`
- `docs/IMPLEMENTATION_STATUS.md`
- `src/task_coordinator/CLAUDE.md`

