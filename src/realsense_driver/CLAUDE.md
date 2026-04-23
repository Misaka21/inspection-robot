# realsense_driver/CLAUDE.md

本包是适配层：使用系统安装的 `realsense2_camera` 驱动，只提供 launch 与配置。

> **在抓取方向中的地位**：**核心传感器**。相机 eye-in-hand（装机械臂末端），为 `grasp_perception` 提供 RGB/Depth/aligned_depth/camera_info/TF。精度上限的关键标定是 `tool0 → camera_link` 的手眼标定。

## 1. 包职责与边界

负责：
- 统一启动参数（namespace / camera_name / config_file）
- 存放可复现的 YAML 配置（`config/realsense.yaml`）

不负责：
- 修改/维护 `realsense2_camera` 源码（交给系统包）
- 手眼标定结果（由 `inspection_bringup` 的静态 TF 发布 `tool0 → camera_link`）

## 2. 数据流（约定）

命名空间约定：
- `/inspection/realsense`
- `camera_name = d435`

抓取场景下游主要使用者：
- `grasp_perception`：订阅 `color/image_raw` + `aligned_depth_to_color/image_raw` + `color/camera_info`，输出抓取候选

典型输出：
- `/inspection/realsense/d435/color/image_raw` (`sensor_msgs/msg/Image`)
- `/inspection/realsense/d435/aligned_depth_to_color/image_raw` (`sensor_msgs/msg/Image`)  ← **抓取主用**
- `/inspection/realsense/d435/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
- `/inspection/realsense/d435/depth/color/points` (`sensor_msgs/msg/PointCloud2`)（可选，FoundationPose 等 6D 路径会用）
- TF：`camera_link` / `camera_color_optical_frame` / `camera_depth_optical_frame`

## 3. 关键参数（抓取场景推荐）

`config/realsense.yaml` 里建议打开：

| 参数 | 推荐值 | 说明 |
|------|-------|------|
| `enable_color` | true | RGB 输入（YOLOv8 使用） |
| `enable_depth` | true | 深度输入 |
| `align_depth.enable` | true | **必开**，让 `grasp_perception` 直接按 RGB 像素查深度 |
| `pointcloud.enable` | true（可选） | 6D 位姿路径使用 |
| `publish_tf` | true | 发布相机内部 TF |
| `color_fps` / `depth_fps` | 15 或 30 | 抓取场景不需要高帧率，15 够用且稳定 |
| `depth_module.emitter_enabled` | true | 近距离抓取场景下深度更稠密 |

## 4. 修改规则

1. 只改 `config/realsense.yaml` 与 `launch/realsense.launch.py`
2. 不要在本包里新增 C++ node（否则职责会混乱）
3. 若接口命名需要调整，优先通过 launch 参数控制（namespace / name）
4. **不要在本包里做手眼标定相关的 TF**——`tool0 → camera_link` 由 `inspection_bringup` 静态 TF 发布

## 5. 文档与 TODO 维护（必须）

- 修改输出接口/namespace/camera_name 时，必须同步更新：本文件、bringup launch、`docs/ARCHITECTURE.md`、仓库根 `TODO.md`
- 新增功能但未实现完：必须把未完成项写入 `TODO.md`（带清晰落点与验收标准）
