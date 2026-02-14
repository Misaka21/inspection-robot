# realsense_driver/CLAUDE.md

本包是适配层：使用系统安装的 `realsense2_camera` 驱动，只提供 launch 与配置。

## 1. 包职责与边界

负责：
- 统一启动参数（namespace/camera_name/config_file）
- 存放可复现的 YAML 配置（`config/realsense.yaml`）

不负责：
- 修改/维护 `realsense2_camera` 源码（交给系统包）

## 2. 数据流（约定）

命名空间约定：
- `/inspection/realsense`
- `camera_name=d435`

典型输出：
- `/inspection/realsense/d435/depth/color/points` (`sensor_msgs/msg/PointCloud2`)

下游使用方：
- `pose_detector` 订阅点云做位姿估计

## 3. 修改规则

1. 只改 `config/realsense.yaml` 与 `launch/realsense.launch.py`
2. 不要在本包里新增 C++ node（否则职责会混乱）
3. 若接口命名需要调整，优先通过 launch 参数控制（namespace/name）

