# agv_driver 架构（应用层 Node + 通信封装）

## 1. 设计目标

按项目 README 的约束，采用“少层级但职责清晰”的结构：

- `agv_driver_node`：应用层（ROS 接口 + 轮询调度）
- `agv_client`：AGV 语义层（导航/状态查询接口）
- `agv_transport`：协议通信层（端口路由、TCP 收发、16B 头）

对应关系与 `task.cpp -> agv_send.cpp -> communicator` 类似。

## 2. 目录结构

```text
agv_driver/
├── include/agv_driver/
│   ├── agv_client.hpp
│   └── agv_transport.hpp
├── src/
│   ├── agv_driver_node.cpp
│   ├── agv_client.cpp
│   └── agv_transport.cpp
├── config/
│   └── agv_driver.yaml
├── launch/
│   └── agv_driver.launch.py
└── docs/
    ├── PROJECT_RELATION_AND_AGV_TASKS.md
    ├── AGV_API_SELECTION.md
    └── agv_api_required/
```

## 3. 各层职责

### 3.1 `agv_driver_node`（应用层）

- 订阅：`~/goal_pose`、`~/cmd_vel`
- 发布：`~/current_pose`、`~/status`、`~/odom`
- 发布 TF：`map -> base_link`
- 轮询：定时调用 `agv_client.poll_state()` 并发布 ROS 消息

Node 不直接处理 socket 和协议编解码。

### 3.2 `agv_client`（语义层）

- 对外提供：
  - `send_goal`
  - `send_open_loop_motion`
  - `stop_open_loop_motion`
  - `poll_state`
- 内部完成 1004/1005/1020/1007/1006/1012/1050 等状态聚合
- 将 AGV 返回字段整理为内部 `AgvPollState`

### 3.3 `agv_transport`（通信层）

- 命令号 -> 端口路由
- TCP 长连接管理（按端口复用）
- 16 字节协议头打包/解包
- 请求-响应收发和 `ret_code` 判定

## 4. 端口路由

- `1000~1999 -> 19204`
- `2000~2999 -> 19205`
- `3000~3999 -> 19206`
- `4000~4999 -> 19207`
- `6000~6999 -> 19210`
- `9300/19301 -> 19301`

## 5. ROS 接口

- 节点：`agv_driver_node`
- 命名空间：`/inspection/agv`
- 订阅：
  - `~/goal_pose` (`geometry_msgs/PoseStamped`)
  - `~/cmd_vel` (`geometry_msgs/Twist`)
- 发布：
  - `~/current_pose` (`geometry_msgs/PoseStamped`)
  - `~/status` (`inspection_interface/msg/AgvStatus`)
  - `~/odom` (`nav_msgs/Odometry`)

## 6. 关键收益

- Node 文件长度显著下降，便于维护。
- 协议改动只影响 `agv_transport`，业务改动只影响 `agv_client`。
- 保持少层级（3 层）同时避免“巨石 Node”。
