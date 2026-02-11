# agv_driver

`agv_driver` 是巡检系统中的 AGV 底盘驱动包，职责是把 ROS2 命令转换为 AGV TCP-IP API 请求，并把 AGV 运行状态持续回传为 ROS 话题。

本包只负责“执行与状态反馈”，不负责路径规划与任务编排。

## 1. 目录结构

```text
agv_driver/
├── include/agv_driver/
│   ├── agv_client.hpp
│   └── agv_transport.hpp
├── src/
│   ├── agv_driver_node.cpp
│   ├── agv_client.cpp
│   └── agv_transport.cpp
├── launch/
│   └── agv_driver.launch.py
├── config/
│   └── agv_driver.yaml
└── docs/
    ├── AGV_DRIVER_ARCHITECTURE.md
    ├── AGV_API_SELECTION.md
    ├── PROJECT_RELATION_AND_AGV_TASKS.md
    └── agv_api_required/
```

## 2. 分层设计

本包按三层拆分，避免把协议细节塞进 Node：

1. `agv_driver_node`（应用层）
2. `agv_client`（语义层）
3. `agv_transport`（通信层）

### 2.1 `agv_driver_node`（应用层）

- 订阅 ROS 指令：
  - `~/goal_pose`
  - `~/cmd_vel`
- 发布 ROS 状态：
  - `~/current_pose`
  - `~/odom`
  - `~/status`
- 可选发布 TF：
  - `map -> base_link`
- 定时轮询 AGV 状态并做字段映射。

### 2.2 `agv_client`（语义层）

- 负责具体业务命令：
  - 控制权（4005/4006/1060）
  - 地图切换与载图状态（2022/1022）
  - 重定位（2002/2004/2003/1021）
  - 导航下发（3051）
  - 开环控制（2010/2000）
  - 状态聚合（1004/1005/1020/1007/1006/1012/1050）
- 将多条状态命令聚合为 `AgvPollState`。

### 2.3 `agv_transport`（通信层）

- 维护 `cmd_type -> port` 映射。
- 处理 16B 协议头打包/解包（sync/version/seq/length/type）。
- 按端口复用 TCP 长连接。
- 检查返回 `ret_code`，非 0 视为失败。

## 3. ROS 接口

默认启动命名空间：`/inspection/agv`。

### 3.1 订阅

- `~/goal_pose` (`geometry_msgs/msg/PoseStamped`)
  - AGV 导航目标位姿（`map` 坐标系）
- `~/cmd_vel` (`geometry_msgs/msg/Twist`)
  - 开环调试速度命令；零速度时可转为 `2000` 停止命令

### 3.2 发布

- `~/current_pose` (`geometry_msgs/msg/PoseStamped`)
- `~/odom` (`nav_msgs/msg/Odometry`)
- `~/status` (`inspection_interface/msg/AgvStatus`)

`AgvStatus` 当前字段：

- `connected`
- `arrived`
- `moving`
- `stopped`
- `current_pose`
- `linear_speed`
- `angular_speed`
- `frame_id`
- `battery_percent`
- `error_code`

### 3.3 状态判定规则（代码实现）

- `arrived = has_nav && task_status == 4`
- `stopped = !has_speed || is_stop`
- `moving = connected && !stopped`

`error_code` 优先级：

1. `BOOTSTRAP_PENDING/BOOTSTRAP_FAILED`（若启动引导未完成）
2. `DISCONNECTED`
3. `EMERGENCY`
4. `BLOCKED`
5. `alarm_level`（`FATAL/ERROR/WARNING`）
6. 上一次请求错误字符串
7. `OK`

## 4. 启动引导（Bootstrap）

当 `enable_bootstrap=true` 时，节点启动后会先执行引导流程：

1. 可选抢占控制权（4005；并通过 1060 判断当前 owner）
2. 可选切换地图（2022）
3. 等待地图载入成功（轮询 1022）
4. 可选自动重定位（2002）
5. 等待定位完成（轮询 1021）
6. 老版本可选确认定位（2003，`require_confirm_loc=true` 时）

在引导未完成前，`goal_pose` 会被丢弃并节流告警。

## 5. 命令与端口路由

本包当前使用到的命令：

- 状态端口 `19204`：
  - 1004, 1005, 1006, 1007, 1012, 1020, 1021, 1022, 1050, 1060
- 控制端口 `19205`：
  - 2000, 2002, 2003, 2004, 2010, 2022
- 导航端口 `19206`：
  - 3051
- 配置端口 `19207`：
  - 4005, 4006

路由规则由 `agv_transport::resolve_port()` 固定维护。命令号发错端口会失败。

## 6. 参数说明

参数定义见 `config/agv_driver.yaml` 与 `agv_driver_node.cpp`，核心参数如下：

- 通信参数：
  - `agv_ip`
  - `protocol_version`
  - `request_timeout_ms`
  - `poll_interval_ms`
- 控制参数：
  - `cmd_vel_duration_ms`
  - `stop_on_zero_cmd_vel`
- 坐标参数：
  - `map_frame_id`
  - `base_frame_id`
  - `publish_tf`
- 启动引导参数：
  - `enable_bootstrap`
  - `enable_control_lock`
  - `control_nick_name`
  - `initial_map_name`
  - `auto_relocate`
  - `skip_reloc_if_localized`
  - `require_confirm_loc`
  - `bootstrap_timeout_ms`
  - `bootstrap_poll_interval_ms`
  - `bootstrap_retry_interval_ms`

## 7. 构建与运行

### 7.1 构建

```bash
cd inspection-robot
colcon build --packages-select agv_driver
```

### 7.2 启动

```bash
source install/setup.bash
ros2 launch agv_driver agv_driver.launch.py
```

可选参数：

```bash
ros2 launch agv_driver agv_driver.launch.py \
  namespace:=/inspection/agv \
  params_file:=/absolute/path/to/agv_driver.yaml
```

### 7.3 常用调试命令

```bash
ros2 topic echo /inspection/agv/status
ros2 topic echo /inspection/agv/current_pose
ros2 topic echo /inspection/agv/odom
```

发送导航目标：

```bash
ros2 topic pub /inspection/agv/goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}'
```

发送开环速度：

```bash
ros2 topic pub /inspection/agv/cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

## 8. 与 task_coordinator 的接口契约（推荐）

默认命名空间下，`task_coordinator` 与 AGV 的最小接口如下：

- 下发导航：`/inspection/agv/goal_pose` (`geometry_msgs/msg/PoseStamped`)
- 可选调试速度：`/inspection/agv/cmd_vel` (`geometry_msgs/msg/Twist`)
- 状态门控读取：`/inspection/agv/status` (`inspection_interface/msg/AgvStatus`)

主流程建议时序：

1. `task_coordinator` 下发 `goal_pose`（`frame_id` 必须为 `map`）
2. 轮询 `status`
3. 只有 `status.connected=true && status.arrived=true && status.stopped=true` 才进入机械臂动作
4. 若 `status.error_code != "OK"`，进入异常分支（取消/重试/告警）

可直接复用的门控条件：

```text
agv_ready = connected && arrived && stopped && (error_code == "OK")
```

## 9. 故障排查建议

1. `status.connected=false`：
   - 检查 AGV IP 和端口联通性
   - 检查命令号端口映射是否被改动
2. 持续 `BOOTSTRAP_FAILED`：
   - 先看 `_bootstrap_error` 日志信息
   - 检查地图名、定位状态与控制权配置
3. `arrived` 长时间为 false：
   - 检查导航命令是否真正下发成功（3051 返回）
   - 检查 `1020 task_status` 字段是否更新
4. `stopped` 判断异常：
   - 检查 `1005 is_stop` 和速度字段返回格式

## 10. 相关文档

- 架构说明：`docs/AGV_DRIVER_ARCHITECTURE.md`
- API 筛选：`docs/AGV_API_SELECTION.md`
- 项目关系与职责：`docs/PROJECT_RELATION_AND_AGV_TASKS.md`
- 必需 API 原文档：`docs/agv_api_required/`
- 端口路由总表：`docs/agv_api_required/PORT_ROUTING.md`
