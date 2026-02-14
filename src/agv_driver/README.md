# agv_driver

`agv_driver` 是巡检系统的 AGV 底盘驱动包：接收 ROS2 指令，下发 AGV TCP-IP API，并持续发布底盘状态供 `task_coordinator` 做到位门控。

本包不负责：路径规划、任务编排、上位机接口（网关）。

## 快速开始

构建：

```bash
cd inspection-robot
colcon build --packages-select agv_driver
```

启动：

```bash
source install/setup.bash
ros2 launch agv_driver agv_driver.launch.py
```

指定参数文件/命名空间：

```bash
ros2 launch agv_driver agv_driver.launch.py \
  namespace:=/inspection/agv \
  params_file:=/absolute/path/to/agv_driver.yaml
```

## ROS 接口

默认命名空间：`/inspection/agv`（见 `launch/agv_driver.launch.py`）。

订阅：

- `goal_pose` (`geometry_msgs/msg/PoseStamped`)
  - 导航目标（应为 `map_frame_id` 坐标系）
  - 若 `header.frame_id` 非空且不等于 `map_frame_id`，会被拒绝并节流告警
- `cmd_vel` (`geometry_msgs/msg/Twist`)
  - 开环调试速度
  - `stop_on_zero_cmd_vel=true` 时零速度会转为 `2000` 停止开环

发布：

- `status` (`inspection_interface/msg/AgvStatus`)
  - 每个轮询周期都会发布（断连也会发布 `connected=false`）
- `current_pose` (`geometry_msgs/msg/PoseStamped`)
- `odom` (`nav_msgs/msg/Odometry`)
- TF（可选）：`map_frame_id -> base_frame_id`

说明：

- `current_pose`、`odom`、TF 只有在内部存在有效位姿时才会发布；断连后可能继续发布上一次有效位姿，同时 `status.connected=false`。

### 到位门控（推荐）

`task_coordinator` 建议使用如下门控条件：

```text
agv_ready = connected && arrived && stopped && (error_code == "OK")
```

## 参数（关键项）

完整参数与中文注释见：`config/agv_driver.yaml`。

常用关键项：

- `agv_ip`：AGV 控制器 IP
- `request_timeout_ms`：单次请求超时
- `poll_interval_ms`：状态轮询周期
- `publish_tf`：是否发布 TF
- `map_frame_id` / `base_frame_id`：TF 与输出 pose/odom 的 frame
- `enable_bootstrap`：是否启用启动引导
- `enable_control_lock` / `control_nick_name`：控制权抢占（可选）
- `initial_map_name`：启动时切图（可选）
- `auto_relocate`：启动时自动重定位（可选）
- `log_io` / `log_io_max_chars`：是否打印 TCP 请求/响应（排障用）

## 启动引导（Bootstrap）

当 `enable_bootstrap=true` 时，节点启动后会按文档流程做“接管准备”（控制权/切图/等待载图/重定位/等待定位等）。引导未完成时 `goal_pose` 会被丢弃并节流告警。

## 日志（排障）

默认日志只包含高层事件（启动、bootstrap 结果、下发 goal、轮询失败等）。

如需查看“发了什么/回了什么”，开启：

- `log_io=true`
- `log_io_max_chars=2048`（按需调整）

会输出：

```text
AGV >> cmd=3051 port=19206 seq=12 payload=...
AGV << cmd=3051 port=19206 seq=12 ret_code=0 cost_ms=8 body=...
```

## 深入文档

- 架构与分层：`docs/AGV_DRIVER_ARCHITECTURE.md`
- API 筛选与命令列表：`docs/AGV_API_SELECTION.md`
- 端口路由总表：`docs/agv_api_required/PORT_ROUTING.md`

## （规划中）地图获取能力：支撑 GetNavMap

HMI 的导航视图通常需要“底图 + 分辨率 + 原点”。地图文件与地图元信息在 AGV 控制器上，厂商 TCP API 提供了查询/下载接口：

- `1300 robot_status_map_req`：查询当前载入地图名与 md5、保存的地图列表  
  `docs/agv_api/API/TCP-IP API/机器人状态API/查询机器人载入的地图以及储存的地图.md`
- `4011 robot_config_downloadmap_req`：按 `map_name` 下载 `.smap`（JSON 文本）  
  `docs/agv_api/API/TCP-IP API/机器人配置API/从机器人下载地图.md`
- `.smap` 格式说明（坐标单位为米，地图坐标系即世界坐标系）  
  `docs/agv_api/API/TCP-IP API/机器人配置API/地图格式说明.md`
- 可选 `1513 robot_status_costmap_req`：查询代价地图（Message_Grid）  
  `docs/agv_api/API/TCP-IP API/机器人状态API/查询代价地图.md`

建议实现方式：

1. 在 `agv_driver` 内封装上述厂商 API（保持“厂商协议不外泄”的分层原则）
2. 通过 ROS2 service 把“解析后的 map 元信息 + 底图”提供给 `inspection_gateway`：
   - service：`get_nav_map`（全名通常为 `/inspection/agv/get_nav_map`）
   - 类型：`inspection_interface/srv/GetNavMap`
3. 对外 gRPC 的 `GetNavMap` 由 `inspection_gateway` 实现并做缓存（key 建议用 `map_name + md5`）
