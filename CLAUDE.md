# CLAUDE.md

本仓库当前课题方向：**基于移动协作机械臂的大型工件视觉检测/巡检系统**。

## 工作口径

- 不再按复合机器人抓取方向推进。
- 不再把网页前端作为当前论文/演示主线。
- 不再把 fake driver 仿真作为当前论文/演示主线。
- 保留历史包源码，但主文档、TODO 和新增工作都应围绕巡检方向。

主链路：

```text
AGV 到站位
  -> 机械臂到预设观测位
  -> RealSense 深度测距并微调工作距
  -> 海康工业相机触发拍照
  -> defect_detector 缺陷检测
  -> ROS2 状态/结果输出
```

## 主线包

- `agv_driver`：AGV 底盘 TCP 驱动。
- `arm_driver`：机械臂底层驱动。
- `arm_controller`：MoveIt2 运动控制。
- `realsense_driver`：深度相机，用于工作距微调。
- `hikvision_driver`：工业相机，用于缺陷图像采集。
- `defect_detector`：缺陷检测算法包。
- `task_coordinator`：巡检状态机。
- `inspection_interface`：ROS2 msg/srv。
- `inspection_bringup`：launch 与配置。
- `inspection_supervisor`：健康监控，可选。

非主线保留：

- `inspection_gateway`：历史 REST/WS 网关；当前无网页主线。
- `inspection_sim`：历史 fake driver 联调；当前不作为论文主线。
- `dio_driver`：DIO 通用驱动；巡检主线通常不需要。

## 巡检状态机

```text
IDLE
  -> MOVING_TO_STATION
  -> ARM_PRESET
  -> DEPTH_ADJUST
  -> CAPTURING
  -> 下一站位 / COMPLETED
```

`DEPTH_ADJUST` 是论文算法重点：基于 RealSense 深度图/局部点云估计非平面工件局部工作距，并将距离误差转换成机械臂末端小范围位姿修正。

`CAPTURING` 阶段应先触发海康相机拍照，再调用缺陷检测，最后发布结构化检测结果。

## 编码规则

- 私有成员变量必须以下划线 `_` 开头，禁止 `_` 结尾。
- Node 只做 IO 与调度；算法、协议、业务逻辑应拆到 Core / Service / Adapter。
- 公共 topic/service 用相对名，通过 launch namespace 固定前缀。
- 修改 public ROS API 或数据流时，同步更新 `TODO.md`、`docs/ARCHITECTURE.md`、`docs/IMPLEMENTATION_STATUS.md` 和相关包说明。

