# inspection_bringup/CLAUDE.md

本包负责 launch 与参数编排，目标是：**所有节点统一由 bringup 启动且参数可复现**，避免每个人本地 launch 各写各的。

> 抓取方向下本包承担两个新增职责：
> 1. 统一发布标定相关的静态 TF（`base_link → arm_base`、`tool0 → camera_link`、`tool0 → gripper_tip`）
> 2. 管理抓取 pipeline 所需节点的启动顺序与条件（相机/感知/夹爪/控制/编排/网关）

## 1. 包职责与边界

负责：
- `drivers.launch.py`：驱动集合（RealSense / arm_driver / agv_driver / dio_driver / 静态 TF）
- `system.launch.py`：完整抓取系统（drivers + arm_controller + gripper_driver + grasp_perception + task_coordinator + gateway）
- `config/*.yaml`：集中参数文件
  - `inspection.yaml`（全局参数：namespace / use_agv / use_hikvision / 子系统开关）
  - `mount_calib.yaml`（`base_link → arm_base`）
  - `handeye_calib.yaml`（`tool0 → camera_link`，来自 `easy_handeye2`）
  - `gripper_offset.yaml`（`tool0 → gripper_tip`）
  - 各子系统的 YAML（agv_driver.yaml / arm_driver.yaml / realsense.yaml / dio_driver.yaml / gripper.yaml / grasp_perception.yaml / grasp_tasks.yaml）
- `fastdds_no_shm.xml`（可选，解决部分环境下 shm 问题）

不负责：
- 业务逻辑（不要写任何 C++ node）
- 手眼标定算法本体（由 `easy_handeye2` 或独立标定脚本完成，结果写到 yaml 后由本包发布）

## 2. Launch 设计原则（必须）

1. 参数文件驱动：默认读取 `config/*.yaml`，代码里不硬编码参数
2. 命名空间统一：通过 launch 参数 `namespace:=/inspection/...` 固定对外接口前缀
3. 图像类节点优先组件容器：减少拷贝（RealSense → grasp_perception 建议同容器）
4. 可分阶段启动：drivers 与 system 分离，便于排障
5. 条件启动：通过 yaml 开关决定是否启动 `use_agv` / `use_hikvision` / `use_sim`

## 3. 当前启动拓扑（抓取目标形态）

`drivers.launch.py`（抓取方向目标）：
- `realsense` composable container（namespace `/inspection/realsense/d435`，启用 align_depth）
- `agv_driver`（namespace `/inspection/agv`，可通过 yaml 关闭）
- `arm_driver`（namespace `/inspection/arm`，需要 sudo 跑 EtherCAT，独立终端启动更稳）
- `dio_driver`（namespace `/inspection/dio`）
- 静态 TF：
  - `base_link → arm_base`（从 mount_calib.yaml）
  - `tool0 → camera_link`（从 handeye_calib.yaml）
  - `tool0 → gripper_tip`（从 gripper_offset.yaml）
- hikvision：条件启动（默认关闭）

`system.launch.py`：
- include drivers
- `arm_controller`（MoveIt2 + Pilz LIN plugin）
- `gripper_driver`（薄封装，调 dio/set_output）
- `grasp_perception`（订阅 RealSense，提供 perceive_grasp service）
- `task_coordinator`（读 grasp_tasks.yaml，启动抓取状态机）
- `inspection_gateway`（:8080 REST/WS）
- foxglove_bridge（可选调试）

> 当前 `drivers.launch.py` 仍是**巡检方向的旧拓扑**（含 hikvision composable container、`tool0 → hikvision_frame` 静态 TF）。迁移到抓取拓扑的任务见 `TODO.md` 的 P0 基础设施节。

## 4. 修改规则

- 新增节点一律先加到 `system.launch.py`，同时给出默认参数文件落点
- 若节点对外接口变化，必须同步更新：
  - 包内 `README.md` 或 `CLAUDE.md`
  - `docs/ARCHITECTURE.md` 接口章节
- 标定 yaml 更新（换相机/换机械臂位姿/重新标定）时，必须在 commit message 注明"静态 TF 已重新发布"
- 禁止在 launch 文件里硬编码 DO 线号、相机 SN、AGV IP 等设备参数——一律走 yaml

## 5. 文档与 TODO 维护（必须）

- 任何 launch/topology 变化必须同步更新：本文件、`docs/ARCHITECTURE.md`、仓库根 `TODO.md`
- 新增节点但未接好参数/命名空间：必须在 `TODO.md` 记录并注明验收方式
