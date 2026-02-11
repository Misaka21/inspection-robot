# agv_api_required

本目录是从 `agv_api` 原始文档中筛选出的“AGV 驱动实现最小必需集”。

## 阅读顺序

1. `00_protocol/`：先读协议格式、端口、坐标系
2. `01_bootstrap/`：控制权、地图载入、重定位
3. `02_navigation/`：导航任务下发与暂停/继续/取消
4. `03_status/`：状态查询与推送字段
5. `04_recovery/`：调试控制、急停、清错、错误码

## 分组说明

- `00_protocol`：TCP 协议与报文基础
- `01_bootstrap`：启动期接管和定位准备
- `02_navigation`：执行导航闭环
- `03_status`：输出 ROS `current_pose/status` 所需字段
- `04_recovery`：异常恢复与现场调试

## 注意

- 这里的文件均为从原始 `agv_api` 复制的原文档，未做语义改写。
- 若后续机器人固件版本升级，需按命令号逐项回归这些文档。

