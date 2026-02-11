# AGV API 筛选结果（面向 inspection-robot）

## 1. 筛选原则

本次只保留与 `inspection-robot` 当前目标直接相关的 API：

- 支持 AGV 到点执行（`goal_pose -> 导航 -> 到位判定`）
- 支持 AGV 状态上报（位置、速度、导航状态、电量、故障）
- 支持调试与恢复（开环运动、取消导航、急停、清错）
- 支持系统接管（控制权、地图载入、重定位）

与本课题当前无关的文档（货叉/辊筒/顶升/音频/库位/标定细节等）未纳入精简集。

## 2. 整理位置

已将必需文档从：

- `inspection-robot/src/agv_driver/docs/agv_api/API/TCP-IP API/...`

筛选并复制到：

- `inspection-robot/src/agv_driver/docs/agv_api_required/`

分组如下：

- `00_protocol`：协议基础
- `01_bootstrap`：控制权、地图、重定位
- `02_navigation`：导航下发与导航控制
- `03_status`：状态查询与主动推送
- `04_recovery`：调试/急停/清错/错误码

## 3. 关键 API（按功能）

### 3.1 协议基础

- `API简介.md`
- `API报文结构.md`
- `坐标系.md`
- `注意事项.md`
- `API使用教程.md`
- `TCP client 模式.md`

### 3.2 启动与接管

- `抢占控制权.md`（4005）
- `释放控制权.md`（4006）
- `查询当前控制权所有者.md`（1060）
- `切换载入的地图.md`（2022）
- `查询机器人地图载入状态.md`（1022）
- `重定位.md`（2002）
- `取消重定位.md`（2004）

### 3.3 导航执行

- `路径导航.md`（3051）
- `指定路径导航.md`（3066）
- `暂停当前导航.md`（3001）
- `继续当前导航.md`（3002）
- `取消当前导航.md`（3003）
- `清除指定导航路径.md`（3067）

### 3.4 状态与门控

- `查询机器人位置.md`（1004）
- `查询机器人速度.md`（1005）
- `查询机器人导航状态.md`（1020）
- `查询机器人定位状态.md`（1021）
- `查询机器人任务状态.md`（1110）
- `查询机器人电池状态.md`（1007）
- `查询机器人的被阻挡状态.md`（1006）
- `查询机器人急停状态.md`（1012）
- `查询机器人报警状态.md`（1050）
- `配置机器人推送端口.md`（9300）
- `机器人推送.md`（19301）

### 3.5 调试与恢复

- `开环运动.md`（2010）
- `停止开环运动.md`（2000）
- `软急停.md`（6004）
- `清除机器人当前所有报错.md`（4009）
- `API 错误码.md`
- `报警码.md`

## 4. 与 ROS 接口的最小映射建议

- `/inspection/agv/goal_pose` -> 3051/3066
- `/inspection/agv/cmd_vel` -> 2010（停止用 2000）
- `/inspection/agv/current_pose` -> 1004 或 19301 推送中的 `x/y/angle`
- `/inspection/agv/status.arrived` -> 1020/1110 的任务状态 `COMPLETED`
- `/inspection/agv/status.stopped` -> 1005/19301 的 `is_stop`
- `/inspection/agv/status.error_code` -> 1050（报警）+ `ret_code`（请求返回）

