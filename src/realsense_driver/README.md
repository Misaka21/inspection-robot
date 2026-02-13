# realsense_driver

RealSense 深度相机驱动适配层。

## 架构

本包**不包含源码**，仅提供配置和启动文件。驱动使用系统安装的 `ros-humble-realsense2-camera`。

```
realsense_driver/
├── config/
│   └── realsense.yaml     # 参数配置
├── launch/
│   └── realsense.launch.py # 启动文件
└── README.md
```

## 依赖

- ROS 2 Humble
- `ros-humble-realsense2-camera` (系统包)
- `ros-humble-realsense2-camera-msgs` (系统包)
- `ros-humble-librealsense2` (系统库)

## 启动方式

### 单独启动

```bash
ros2 launch realsense_driver realsense.launch.py
```

### 通过 bringup 统一启动

```bash
ros2 launch inspection_bringup drivers.launch.py
```

## 配置

修改 `config/realsense.yaml`：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| enable_color | 启用彩色图像 | true |
| enable_depth | 启用深度图像 | true |
| align_depth.enable | 深度-彩色对齐 | true |
| pointcloud.enable | 启用点云 | true |
| publish_tf | 发布 TF | true |

## ROS 接口

- **命名空间**：`/inspection/realsense`
- **相机名**：`d435`

常用话题：

| 话题 | 类型 |
|------|------|
| `/inspection/realsense/d435/color/image_raw` | Image |
| `/inspection/realsense/d435/depth/image_rect_raw` | Image |
| `/inspection/realsense/d435/depth/color/points` | PointCloud2 |
