# realsense_driver

`realsense_driver` 是巡检系统中的深度相机驱动层目录。这里**不实现自研驱动节点**，而是直接 vendoring Intel/RealSense 官方 ROS 2 驱动：

- `realsense2_camera`：相机数据发布（image/depth/pointcloud/TF 等）
- `realsense2_camera_msgs`：驱动相关 msg/srv/action 定义

本目录的职责是“提供官方驱动源码 + 让 bringup 能统一启动与配置”；不负责算法层逻辑（位姿估计/点云处理/缺陷检测等）。

## 1. 目录结构

```text
realsense_driver/
├── realsense2_camera/          # upstream 驱动包（已做最小化裁剪）
├── realsense2_camera_msgs/     # upstream 消息定义包
├── _extras/                    # 上游附加内容（不参与默认构建/CI）
│   ├── realsense2_camera_test/ # 上游测试集合（偏功能验证）
│   └── realsense2_camera/      # 上游脚本/更多示例/变更记录等
└── README.md
```

说明：
- `realsense2_camera` 的标准启动入口在 `realsense2_camera/launch/rs_launch.py`。
- 为了不把上游的集成/真机测试混入本项目“单元测试口径”，上游 `test/`、部分 `scripts/`、部分 `examples/` 已移动到 `_extras/` 做备份保留。

## 2. 启动方式（推荐）

本项目通过 `inspection_bringup` 统一启动 RealSense（统一命名空间与参数文件）：

```bash
source install/setup.bash
ros2 launch inspection_bringup drivers.launch.py
```

`drivers.launch.py` 当前会 include 官方 `rs_launch.py`，并传入：

- `camera_namespace:=inspection/realsense`
- `camera_name:=d435`
- `config_file:=<inspection_bringup>/config/realsense.yaml`

对应配置文件在：`inspection_bringup/config/realsense.yaml`。

## 3. ROS 接口（默认约定）

官方驱动大量 topic 使用 `~/`（私有命名空间）。在本项目默认启动参数下：

- **命名空间**：`/inspection/realsense`
- **节点名（camera_name）**：`d435`
- **完整话题前缀通常为**：`/inspection/realsense/d435/...`

常用发布（以默认打开 color+depth+align+pointcloud 为例）：

- `~/color/image_raw` (`sensor_msgs/msg/Image`)
- `~/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
- `~/depth/image_rect_raw` (`sensor_msgs/msg/Image`)
- `~/depth/camera_info` (`sensor_msgs/msg/CameraInfo`)
- `~/depth/color/points` (`sensor_msgs/msg/PointCloud2`)
- `~/aligned_depth_to_color/image_raw`（当 `align_depth.enable=true`）
- `~/aligned_depth_to_color/camera_info`（当 `align_depth.enable=true`）

TF：
- `publish_tf=true` 时，驱动会发布相机内部 TF（frame 命名与 `camera_name`/`base_frame_id`/`tf_prefix` 等参数有关）。

## 4. 默认参数（项目侧）

本项目在 `inspection_bringup/config/realsense.yaml` 里给出了一组“够用的默认值”：

```yaml
enable_color: true
enable_depth: true
enable_infra: false
enable_sync: true
align_depth.enable: true
pointcloud.enable: true
publish_tf: true
tf_publish_rate: 10.0
initial_reset: false
```

如需调整分辨率/FPS、IMU、对齐策略等，请以官方 `rs_launch.py` 的参数列表为准，并在该 YAML 中覆盖。

## 5. 功能性验证（非算法测试）

这里保留少量“功能性测试入口”，用于验证驱动是否正常发布数据与 TF，不纳入算法层测试。

### 5.1 Frame Latency 工具节点（可选）

`realsense2_camera` 内置 `frame_latency` 工具节点（订阅指定 topic 并打印端到端延迟）。默认不编译，需在构建时显式开启：

```bash
colcon build --packages-select realsense2_camera --cmake-args -DBUILD_TOOLS=ON
source install/setup.bash
ros2 run realsense2_camera realsense2_frame_latency_node --ros-args \
  -p topic_name:=/inspection/realsense/d435/depth/color/points \
  -p topic_type:=points
```

### 5.2 上游测试与脚本（保留在 _extras）

- 上游测试：`_extras/realsense2_camera_test/`
- 上游脚本/更多示例：`_extras/realsense2_camera/`

它们更偏 B/C 级验证（rosbag 回放/真机联调），不建议混入本项目 A 级单元测试（见仓库主 README 的测试规范）。

## 6. 依赖

- ROS 2 Humble
- `librealsense2`（系统库）
- 其它依赖以 `realsense2_camera/package.xml` 为准（OpenCV、cv_bridge、image_transport 等）。

## 7. Upstream / License

`realsense2_camera` 与 `realsense2_camera_msgs` 来自 RealSense 官方 ROS 2 驱动仓库，License 为 Apache-2.0（以各包 `package.xml` 声明为准）。

