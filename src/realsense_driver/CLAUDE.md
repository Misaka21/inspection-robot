# realsense_driver/CLAUDE.md

本包是适配层：使用系统安装的 `realsense2_camera` 驱动，只提供 launch 与配置。

## 1. 在巡检方向中的职责

RealSense 安装在机械臂末端，当前主用途是为 `task_coordinator` 的 `DEPTH_ADJUST` 阶段提供深度图、相机内参和相机 TF，用于估计工业相机拍照前的局部工作距。

负责：

- 统一启动参数（namespace / camera_name / config_file）。
- 提供 aligned depth、camera_info、可选点云。
- 存放可复现的 YAML 配置。

不负责：

- 修改 `realsense2_camera` 源码。
- 在本包中实现深度微调算法。
- 发布 `tool0 -> camera_link` 手眼外参；外参应由 `inspection_bringup` 静态 TF 或 URDF 提供。

## 2. 数据流

命名空间：

- `/inspection/realsense`
- `camera_name = d435`

典型输出：

- `/inspection/realsense/d435/color/image_raw`
- `/inspection/realsense/d435/aligned_depth_to_color/image_raw`
- `/inspection/realsense/d435/color/camera_info`
- `/inspection/realsense/d435/depth/color/points`（可选）
- TF：`camera_link` / `camera_color_optical_frame` / `camera_depth_optical_frame`

下游：

- `task_coordinator` / `DepthAdjustCore`：订阅 aligned depth 和 camera_info，估计工作距。

## 3. 推荐参数

| 参数 | 推荐值 | 说明 |
|---|---|---|
| `enable_color` | true | 便于调试 ROI |
| `enable_depth` | true | 工作距微调必须 |
| `align_depth.enable` | true | 让深度与彩色像素对齐 |
| `pointcloud.enable` | true（可选） | 局部点云算法可用 |
| `publish_tf` | true | 发布相机内部 TF |
| `color_fps` / `depth_fps` | 15 或 30 | 巡检场景稳定优先 |

## 4. 修改规则

- 只改 `config/realsense.yaml` 与 `launch/realsense.launch.py`。
- 不要在本包新增算法 node。
- 接口命名优先通过 launch 参数控制。
- 输出接口变更必须同步更新 `docs/ARCHITECTURE.md`、`TODO.md` 和 `task_coordinator` 文档。

