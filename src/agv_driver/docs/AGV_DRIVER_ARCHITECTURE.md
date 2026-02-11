# agv_driver 架构整理

## 1. 当前现状

当前包目录为：

- `inspection-robot/src/agv_driver/`
  - `CMakeLists.txt`、`package.xml`
  - `src/`、`include/`、`launch/`、`config/`（目前为空）
  - `docs/`（已整理 AGV API 文档）

说明：`CMakeLists.txt` 已声明 `agv_driver_node`，但实现文件尚未落地。

## 2. 目标职责边界

`agv_driver` 只做三件事：

- TCP 协议通信（与 AGV 控制器交互）
- ROS 接口适配（订阅目标、发布状态/位姿/TF）
- 运行态管理（状态机、重试、超时、故障恢复）

不在该包内做：

- 路径规划优化（由 `path_planner` 完成）
- 任务编排（由 `task_coordinator` 完成）

## 3. 推荐目录架构（实现蓝图）

```text
agv_driver/
├── include/agv_driver/
│   ├── agv_driver_node.hpp         # ROS2 Node 对外接口
│   ├── agv_client.hpp              # AGV 业务 API 封装（按命令号）
│   ├── tcp_session.hpp             # TCP 连接、收发、重连
│   ├── protocol_codec.hpp          # 16B头 + JSON 编解码
│   ├── agv_state_mapper.hpp        # AGV字段 -> AgvStatus 映射
│   └── types.hpp                   # 内部状态/枚举定义
├── src/
│   ├── agv_driver_node.cpp
│   ├── agv_client.cpp
│   ├── tcp_session.cpp
│   ├── protocol_codec.cpp
│   └── agv_state_mapper.cpp
├── config/
│   └── agv_driver.yaml             # IP/端口/超时/轮询周期/阈值
├── launch/
│   └── agv_driver.launch.py
└── docs/
    ├── PROJECT_RELATION_AND_AGV_TASKS.md
    ├── AGV_API_SELECTION.md
    └── agv_api_required/
```

## 4. 节点内模块分工

- `AgvDriverNode`
  - ROS topic/service/timer 的统一入口
  - 维护“目标命令队列”和“状态发布节奏”
- `AgvClient`
  - 提供语义化方法：`Relocate`、`NavigateToPose`、`CancelTask`、`QueryNavState` 等
  - 屏蔽命令号细节（如 2002/3051/1020）
- `TcpSession`
  - 管理多端口连接（19204/19205/19206/19207/19210/19301）
  - 请求-响应关联（序号）与超时控制
- `ProtocolCodec`
  - 实现 16 字节报文头编解码与 JSON 序列化
- `AgvStateMapper`
  - 将 1004/1005/1020/1050/19301 等数据统一映射为 `AgvStatus`

## 5. ROS 接口架构

### 5.1 订阅

- `~/goal_pose` (`geometry_msgs/PoseStamped`)
- `~/cmd_vel` (`geometry_msgs/Twist`, 可选调试)

### 5.2 发布

- `~/current_pose` (`geometry_msgs/PoseStamped`)
- `~/status` (`inspection_interface/msg/AgvStatus`)
- `~/odom` (`nav_msgs/Odometry`, 若可计算/可获取)
- TF: `map -> base_link`

### 5.3 状态门控定义

- `arrived`: 导航任务状态为 `COMPLETED`
- `stopped`: 速度状态 `is_stop=true`
- `moving`: 任务状态 `RUNNING` 且 `is_stop=false`

该定义与 `task_coordinator` 的门控条件 `arrived && stopped` 保持一致。

## 6. 驱动状态机（建议）

```text
DISCONNECTED
  -> CONNECTING
  -> READY
  -> LOCALIZING
  -> NAVIGATING
  -> PAUSED
  -> ERROR
```

关键转移：

- `READY -> LOCALIZING`：地图载入成功后触发重定位
- `LOCALIZING -> READY`：定位状态成功
- `READY -> NAVIGATING`：收到 `goal_pose`
- `NAVIGATING -> READY`：到位且静止
- `* -> ERROR`：急停/报警/通信失败/超时

## 7. API 映射建议（最小集）

- 初始化与接管：`4005/4006/1060/1022/2022/2002/2004`
- 导航控制：`3051/3066/3001/3002/3003/3067`
- 状态查询：`1004/1005/1020/1021/1110/1007/1006/1012/1050`
- 推送：`4091`（配置 API）或 `9300`（推送 API） + `19301`（推送消息）
- 容错：`2010/2000/6004/4009`

## 8. 端口路由约束（实现时必须硬编码）

| 端口 | 命令号范围/命令 |
| --- | --- |
| `19204` | `1000~1999`（本项目用到：`1004,1005,1006,1007,1012,1020,1021,1022,1050,1060,1110`） |
| `19205` | `2000~2999`（本项目用到：`2000,2002,2004,2010,2022`） |
| `19206` | `3000~3999`（本项目用到：`3001,3002,3003,3051,3066,3067`） |
| `19207` | `4000~4999`（本项目用到：`4005,4006,4009,4091`） |
| `19210` | `6000~6999`（本项目用到：`6004`） |
| `19301` | 推送通道（`9300` 在部分版本用于配置推送；机器人主动推送消息类型 `19301`） |

工程建议：

- 在 `AgvClient` 内做 `cmd_type -> port` 映射表，不允许调用方直接指定端口。
- 收到“报文类型不属于该端口”的错误时，直接标记为程序错误而不是业务错误。
- 推送连接与请求响应连接分离，避免互相阻塞。
- 推送配置入口存在 `4091` 与 `9300` 两种时，使用版本配置项显式选择，不要写死其一。
