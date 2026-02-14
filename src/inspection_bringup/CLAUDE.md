# inspection_bringup/CLAUDE.md

本包负责 launch 与参数编排，目标是：**所有节点统一由 bringup 启动且参数可复现**，避免每个人本地 launch 各写各的。

## 1. 包职责与边界

负责：
- `drivers.launch.py`：驱动集合（相机/TF/必要 adapter）
- `system.launch.py`：完整系统（drivers + perception/planning/coordinator + 可视化）
- `config/*.yaml`：集中参数文件

不负责：
- 业务逻辑（不要写任何 C++ node）

## 2. Launch 设计原则（必须）

1. 参数文件驱动：默认读取 `config/*.yaml`，代码里不硬编码参数
2. 命名空间统一：通过 launch 参数 `namespace:=/inspection/...` 固定对外接口前缀
3. 图像类节点优先组件容器：减少拷贝（如 `hikvision_driver`）
4. 可分阶段启动：drivers 与 system 分离，便于排障

## 3. 当前启动拓扑（骨架）

`drivers.launch.py`：
- hikvision composable container（namespace `/inspection/hikvision`）
- realsense2_camera（namespace `/inspection/realsense/d435`）
- 静态 TF 示例（tcp->hikvision_frame）

`system.launch.py`：
- include drivers
- include agv_driver/arm_driver/arm_controller/pose_detector/path_planner/defect_detector/task_coordinator
- foxglove_bridge

## 4. 修改规则

- 新增节点一律先加到 `system.launch.py`，同时给出默认参数文件落点
- 若节点对外接口变化，必须同步更新：
  - 包内 `README.md` 或 `CLAUDE.md`
  - `docs/ARCHITECTURE.md` 接口章节

