# dio_driver/CLAUDE.md

本文件约束 `dio_driver` 的架构与数据流。

## 1. 包职责与边界

负责：
- 通过 Linux libgpiod 读写工控机板载 DIO（研华 AGX 载板，TCA9539 I2C GPIO 扩展）
- 把 GPIO 能力封装为稳定 ROS2 接口（topic/service），供 `task_coordinator`/网关使用

不负责：
- 任务编排（`task_coordinator`）
- AGV 上的 DI/DO（那是 AGV 厂商协议，在 `agv_driver` 里）

## 2. Public ROS API

默认命名空间：`/inspection/dio`

发布：
- `status` (`inspection_interface/msg/DioStatus`) — DI/DO 状态快照，默认 20Hz

服务：
- `set_output` (`inspection_interface/srv/SetDioOutput`) — 设置单路 DO

参数：
- `chip_path` (string, "/dev/gpiochip3") — gpiochip 设备路径
- `do_lines` (int[], [0,1,2,3]) — DO 对应的 GPIO line 号
- `di_lines` (int[], [8,9,10,11]) — DI 对应的 GPIO line 号
- `poll_interval_ms` (int, 50) — 轮询间隔
- `use_fake` (bool, false) — 历史参数，true 时使用内存 mock，无需硬件；当前巡检主线通常不需要 DIO

## 3. 分层架构

```
Node 层 (dio_node.py)      — ROS IO: pub/sub/srv/timer，不 import gpiod
Adapter 层 (gpio_adapter.py) — 所有 gpiod 调用隔离在此
```

- `GpioAdapter`：真实硬件，使用 gpiod v2 API
- `FakeGpioAdapter`：内存 mock，`use_fake=true` 时使用（仅调试）

## 4. 硬件引脚映射（研华 AGX / gpiochip3 / TCA9539）

| 功能 | line | 名称 |
|------|------|------|
| DO0 | 0 | EXPA2_GPO_0 |
| DO1 | 1 | EXPA2_GPO_1 |
| DO2 | 2 | EXPA2_GPO_2 |
| DO3 | 3 | EXPA2_GPO_3 |
| DI0 | 8 | EXPA2_GPI_0 |
| DI1 | 9 | EXPA2_GPI_1 |
| DI2 | 10 | EXPA2_GPI_2 |
| DI3 | 11 | EXPA2_GPI_3 |

## 5. 权限

非 root 运行需要将用户加入 gpio 组：
```bash
sudo usermod -aG gpio $USER
```

## 6. 文档与 TODO 维护（必须）

- 修改 public ROS API 时，必须同步更新：本文件、`docs/ARCHITECTURE.md`、仓库根 `TODO.md`
