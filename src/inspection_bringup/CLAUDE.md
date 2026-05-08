# inspection_bringup/CLAUDE.md

本包负责 launch 与参数编排，目标是让巡检系统可复现启动。

## 1. 当前主线

巡检方向启动目标：

- `agv_driver`
- `arm_driver`
- `realsense_driver`
- `hikvision_driver`
- `arm_controller`
- `defect_detector`
- `task_coordinator`
- 静态 TF：`base_link -> arm_base`、`tool0 -> hikvision_frame`、`tool0 -> camera_link`

当前不把网页网关和 fake driver 作为主线启动项。

## 2. 包职责

负责：

- `drivers.launch.py`：驱动集合与静态 TF。
- `system.launch.py`：完整巡检系统。
- `config/*.yaml`：相机、站位、驱动、控制参数。
- `inspection_stations.yaml`：巡检站位配置。

不负责：

- 业务逻辑。
- 缺陷检测算法。
- 标定算法本体。
- fake driver 仿真。

## 3. Launch 设计原则

- 参数文件驱动，避免硬编码。
- 命名空间统一。
- drivers 与 system 分离，便于排障。
- 图像类节点可用组件容器减少拷贝。
- 条件启动只用于可选节点；主线默认面向真机。

## 4. 巡检拓扑

`drivers.launch.py`：

- 海康工业相机 container。
- RealSense 驱动。
- AGV 驱动。
- 机械臂驱动。
- 静态 TF。

`system.launch.py`：

- include drivers。
- `arm_controller`。
- `defect_detector`。
- `task_coordinator`，参数 `stations_file` 指向 `inspection_stations.yaml`。
- `inspection_supervisor` 可选。

## 5. 修改规则

- 新增节点先明确是否属于巡检主线。
- 设备参数（相机 SN、AGV IP、标定值）必须走 YAML。
- 修改 launch/topology 必须同步更新 `README.md`、`docs/ARCHITECTURE.md`、`docs/IMPLEMENTATION_STATUS.md`、`TODO.md`。

