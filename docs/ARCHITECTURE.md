# inspection-robot 系统架构文档

## 1. 项目概述

基于移动协作机械臂的大型工件视觉检测系统（ROS2 Humble）。

- **硬件组成**：
  - 仙宫智能 AGV（移动平台，SLAM 导航）
  - 大族 E05 协作机械臂（6自由度）
  - Intel RealSense 深度相机（末端安装）
  - 海康工业相机（末端安装）

- **技术方案**：AGV定点巡检 + 机械臂预设位姿 + 深度相机测距微调

## 2. 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 (Jammy) |
| ROS 版本 | ROS 2 Humble |
| 编译器 | GCC 11+ (C++17) |
| Python | 3.10+ |

## 3. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                     前端 (inspection-site)                        │
│                  React HMI (Engineer/Operator)                    │
└────────────────────────────┬────────────────────────────────────┘
                             │ REST API (/api/v1/*) + WebSocket (/ws)
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                     inspection_gateway (AGX)                      │
│        对外：FastAPI REST/WS  对内：调用 ROS2 节点/服务/话题         │
└────────────────────────────┬────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                    inspection_interface                          │
│              消息/服务定义 (对齐网关语义)                         │
└────────────────────────────┬────────────────────────────────────┘
                             │
        ┌────────────────────┼────────────────────┐
        ▼                    ▼                    ▼
┌───────────────┐  ┌─────────────────┐  ┌─────────────────┐
│   驱动层      │  │    控制层       │  │    协调层       │
│               │  │                 │  │                 │
│ elfin_sdk    │  │ arm_controller  │  │task_coordinator│
│  (EtherCAT)  │  │  (MoveIt2)      │  │  (状态机)       │
│ arm_driver   │  │  (依赖 elfin_sdk)│  │                 │
│ agv_driver   │  └─────────────────┘  └─────────────────┘
│ hikvision_   │         │                    │
│   driver     │         ▼                    │
│ realsense_   │  ┌─────────────────┐        │
│   (适配层)   │  │    算法层       │        │
│               │  │                 │        │
│               │  │ defect_detector│◄───────┘
│               │  │                 │
└───────────────┘  └─────────────────┘
        │                                        │
        ▼                                        │
┌───────────────────────────────────────────────────────┐
│              inspection_bringup / inspection_supervisor │
│                  启动管理 / 系统监控                   │
└───────────────────────────────────────────────────────┘
```

说明：
- `inspection_gateway` 在本仓库中以 ROS2 包形式存在（建议部署在 AGX）：`src/inspection_gateway/`
- 前端不直接连接 ROS2，所有对外 API 以 `inspection_gateway/api/models.py`（Pydantic v2）为准

## 4. 功能包索引

### 4.1 驱动层 (Drivers)

| 包名 | 职责 | 备注 |
|------|------|------|
| `elfin_sdk` | Elfin 机械臂底层驱动 | 包含 elfin_ethercat_driver, soem_ros2, elfin_core |
| `arm_driver` | 机械臂 EtherCAT 驱动 | 调用 elfin_sdk |
| `agv_driver` | AGV 底盘 TCP 驱动 | `/inspection/agv` |
| `hikvision_driver` | 海康工业相机驱动 | `/inspection/hikvision` |
| `realsense_driver` | RealSense 相机适配层 | 使用系统包 ros-humble-realsense2-camera |

### 4.2 控制层 (Control)

| 包名 | 职责 | 备注 |
|------|------|------|
| `arm_controller` | MoveIt2 运动控制 | 运行时依赖 `elfin5_ros2_moveit2` |

### 4.3 算法层 (Algo)

| 包名 | 职责 | 命名空间 |
|------|------|----------|
| `defect_detector` | 图像缺陷检测 | `/inspection/perception` |

### 4.4 协调层 (Coordination)

| 包名 | 职责 | 命名空间 |
|------|------|----------|
| `task_coordinator` | 任务状态机管理 | `/inspection` |

### 4.5 基础设施 (Infra)

| 包名 | 职责 |
|------|------|
| `inspection_interface` | 消息/服务定义 |
| `inspection_gateway` | Web 网关（FastAPI REST/WS，HMI ↔ ROS2 桥接） |
| `inspection_bringup` | 启动管理 |
| `inspection_supervisor` | 系统监控 |
| `inspection_sim` | 无硬件联调（fake drivers + sim launch） |

## 5. API 优先级

1. **外部契约层（最高）**：`inspection_gateway/api/models.py`（Pydantic v2）+ OpenAPI 自动文档
2. **机器人内部编排层**：`inspection_interface`（对齐网关语义）
3. **设备厂商层（最低）**：AGV TCP API / 相机 SDK（仅驱动内部使用）

## 6. 状态机

```
IDLE ──▶ MOVING_TO_STATION ──▶ ARM_PRESET ──▶ DEPTH_ADJUST ──▶ CAPTURING ──▶ (下一站 or COMPLETED)
  ▲              │                  │               │               │
  │              ▼                  ▼               ▼               ▼
  │           PAUSED ◀────────── FAILED ◀──────────────────────────┘
  │              │
  └──────────────┘ (RESUME/STOP)
```

| 状态 | 说明 |
|------|------|
| IDLE | 空闲，等待任务 |
| MOVING_TO_STATION | AGV 导航到站位（YAML 配置驱动） |
| ARM_PRESET | 机械臂移动到预设位姿（MoveToJoints 服务） |
| DEPTH_ADJUST | 深度相机测距，机械臂沿法线微调到目标距离 |
| CAPTURING | 拍照 + 缺陷检测 |
| PAUSED | 任务暂停 |
| COMPLETED | 巡检完成 |
| FAILED | 巡检失败 |
| STOPPED | 任务停止 |

## 7. ROS 接口

### 7.1 agv_driver

**订阅**：
- `goal_pose` (PoseStamped) - 导航目标（map坐标系）
- `cmd_vel` (Twist) - 手动速度指令

**发布**：
- `status` (AgvStatus) - 底盘状态
- `current_pose` (PoseStamped) - 当前位姿
- `odom` (Odometry) - 里程计

**服务**：
- `get_nav_map` (`inspection_interface/srv/GetNavMap`) - 提供导航底图元信息/底图数据（供 inspection_gateway 对外实现 GetNavMap）

### 7.2 arm_driver

**订阅**：
- `joint_cmd` (JointState) - 关节指令

**发布**：
- `/joint_states` (JointState) - 关节状态
- `status` (ArmStatus) - 机械臂状态

**服务**：
- `enable` - 使能
- `disable` - 禁用
- `clear_fault` - 清除故障
- `stop` - 停止

### 7.3 hikvision_driver

**发布**：
- `image_raw` (Image) - 原始图像
- `camera_info` (CameraInfo) - 相机参数

**服务**：
- `trigger_capture` (`std_srvs/srv/Trigger`) - 触发拍照

### 7.4 realsense_driver

> 注意：本包是适配层，使用系统包 ros-humble-realsense2-camera，接口由系统包定义。

**命名空间**：`/inspection/realsense`
**相机名**：`d435`

**发布**（常用）：
- `~/color/image_raw` (Image) - 彩色图像
- `~/depth/image_rect_raw` (Image) - 深度图像
- `~/depth/color/points` (PointCloud2) - 点云
- `~/aligned_depth_to_color/image_raw` - 对齐后的深度图

**TF**：
- 发布相机内部 TF（通过 publish_tf 参数控制）

### 7.5 task_coordinator

**发布**：
- `/inspection/state` (SystemState) - 系统状态
- `/inspection/agv/goal_pose` (PoseStamped) - AGV 目标点

**订阅**：
- `/inspection/agv/status` (AgvStatus) - AGV 状态（用于 arrived/stopped 门控）
- `/inspection/arm/status` (ArmStatus) - 机械臂状态
- `/inspection/realsense/d435/aligned_depth_to_color/image_raw` (Image) - 深度图

**服务客户端**：
- `/inspection/arm_control/move_to_joints` (MoveToJoints) - 机械臂预设位姿
- `/inspection/arm_control/move_to_pose` (MoveToPose) - 深度微调
- `/inspection/perception/detect_defect` (Trigger) - 触发缺陷检测

**服务**：
- `/inspection/start` - 启动巡检
- `/inspection/stop` - 停止巡检
- `/inspection/pause` - 暂停巡检
- `/inspection/resume` - 恢复巡检
- `/inspection/get_status` - 获取状态

**配置**：
- `stations_file` 参数指向 `inspection_stations.yaml`（YAML 配置驱动的站位序列）

### 7.6 defect_detector

**订阅**：
- `/inspection/hikvision/image_raw` (Image)

**发布**：
- `result` (DefectInfo) - 检测结果

**服务**：
- `detect_defect` - 触发检测

## 8. 消息/服务定义

为了避免文档与实现漂移，本仓库不在此处复制 `.msg/.srv` 的字段列表。

- 机器人内部接口：`src/inspection_interface/msg`、`src/inspection_interface/srv`
- 对外 API 模型：`src/inspection_gateway/inspection_gateway/api/models.py`

## 9. TF 树

```
map (SLAM全局坐标系)                    ← agv_driver 发布 (map → base_link)
 └─ base_link (AGV底盘)
      └─ arm_base (机械臂基座)          ← ⚠ 静态标定，当前缺失，需补充
           └─ ... (URDF 链)
                └─ tool0 (末端)         ← URDF 正解
                     └─ hikvision_frame ← drivers.launch.py 静态 TF (tool0 → hikvision_frame)
```

## 10. 数据流

### 10.1 初始化阶段
```
启动 inspection_bringup → 加载配置 → 驱动连接 → TF 发布
```

### 10.2 巡检执行（配置驱动）
```
for station in stations.yaml:
    task_coordinator → agv_driver (goal_pose)        # MOVING_TO_STATION
    wait arrived && stopped
    task_coordinator → arm_controller (MoveToJoints)  # ARM_PRESET
    realsense_driver → task_coordinator (depth)        # DEPTH_ADJUST
    task_coordinator → arm_controller (MoveToPose)     # 微调
    task_coordinator → defect_detector (trigger)       # CAPTURING
```

## 11. 启动方式

```bash
# 驱动层（相机）
ros2 launch inspection_bringup drivers.launch.py

# 完整系统
ros2 launch inspection_bringup system.launch.py

# 单包启动
ros2 launch agv_driver agv_driver.launch.py
ros2 launch arm_driver arm_driver.launch.py
ros2 launch hikvision_driver hikvision_driver.launch.py
ros2 launch arm_controller arm_controller.launch.py
```

## 12. 代码规范

### 12.1 格式化
- 规则文件：`.clang-format`

### 12.2 静态检查
- 规则文件：`.clang-tidy`

### 12.3 命名规则
- **private 成员变量必须以下划线 `_` 开头**
- 示例：`_frame_id`、`_retry_count`

### 12.4 包结构
```
<package_name>/
├── include/<package>/     # 头文件
├── src/                   # 源文件
├── config/                # 参数配置
├── launch/                # 启动文件
├── test/                  # 单元测试 (C++)
├── tests/                 # 单元测试 (Python)
└── package.xml
```

### 12.5 话题命名
- **跨包/对外 ROS API**：使用相对话题名 `topic`（不以 `/` 或 `~` 开头），由 launch 的 `namespace` 固定前缀
- `~/` 仅用于节点私有的内部调试话题
- 避免硬编码绝对话题名
- 命名空间：`/inspection/*`
- 对外 REST/WS API：以 `inspection_gateway/api/models.py` 为准

## 13. Launch 文件规范（参考 rm_bringup & radar_bring2）

### 13.1 设计原则

| 特性 | 说明 | 参考来源 |
|------|------|----------|
| 参数文件驱动 | 使用 YAML 配置文件，不在代码中硬编码 | rm_bringup |
| 组件容器 | 图像类节点使用 ComposableNodeContainer 实现进程内通信 | 两者都有 |
| 延迟启动 | 使用 TimerAction 避免竞争条件 | rm_bringup |
| 条件编译 | 根据参数决定启动哪些节点 | rm_bringup |
| 命名空间 | 使用 PushRosNamespace 统一管理 | rm_bringup |
| TF 变换 | 使用函数封装静态变换发布器 | radar_bringup |
| Debug 模式 | 支持开发/生产模式切换 | radar_bringup |

### 13.2 目录结构

```
inspection_bringup/
├── launch/
│   ├── system.launch.py          # 主启动文件（包含所有层）
│   ├── drivers.launch.py         # 驱动层（相机、静态 TF）
│   ├── camera_only.launch.py     # 仅相机（调试用）
│   └── ...
├── config/
│   ├── inspection.yaml           # 全局参数
│   └── realsense.yaml            # RealSense 参数
└── package.xml
```

### 13.3 参数文件示例

```yaml
# config/inspection.yaml（实际参数示例）
namespace: "inspection"
agv:
  ip: "192.168.1.10"
  bootstrap_timeout_ms: 60000
arm:
  ethernet_name: "enp3s0"
  state_publish_rate_hz: 100.0
hikvision:
  use_trigger_mode: true
  exposure_time: 5000.0
```

### 13.4 组件容器示例

```python
# 将相机和检测器放到同一容器，实现零拷贝
def get_algorithm_container():
    return ComposableNodeContainer(
        name='algorithm_container',
        namespace='inspection',
        package='rclcpp_components',
        executable='component_container_isolated',
        arguments=['--use_multi_threaded_executor'],
        composable_node_descriptions=[
            ComposableNode(
                package='defect_detector',
                plugin='defect_detector::DefectDetectorNode',
                name='defect_detector',
                namespace='inspection/perception',
                parameters=[node_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )
```

### 13.5 延迟启动示例

```python
# 延迟2秒启动，等待其他节点就绪
from launch.actions import TimerAction

delay_algorithm_node = TimerAction(
    period=2.0,
    actions=[algorithm_container],
)
```

### 13.6 TF 变换封装示例

```python
def get_tf_broadcaster(cali: list, parent_frame: str, child_frame: str):
    """封装静态 TF 发布器"""
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=parent_frame + '_to_' + child_frame,
        arguments=[
            '--x', str(cali[0]),
            '--y', str(cali[1]),
            '--z', str(cali[2]),
            '--qx', str(cali[3]),
            '--qy', str(cali[4]),
            '--qz', str(cali[5]),
            '--qw', str(cali[6]),
            '--frame-id', parent_frame,
            '--child-frame-id', child_frame,
        ],
    )
```

### 13.7 Debug 模式示例

```python
debug = False  # 生产环境设为 False

def get_detection_container():
    if not debug:
        # 正式环境：使用组件容器（进程内通信）
        return ComposableNodeContainer(...)
    else:
        # 调试环境：使用独立节点（便于查看日志）
        return Node(...)
```

### 13.8 完整示例结构

```python
def generate_launch_description():
    # 1. 加载参数
    launch_params = yaml.safe_load(open(...))

    # 2. 定义节点（使用函数封装）
    drivers = get_drivers_container()
    algorithms = get_algorithm_container()

    # 3. 延迟启动
    delay_drivers = TimerAction(period=1.0, actions=[drivers])
    delay_algorithms = TimerAction(period=2.0, actions=[algorithms])

    # 4. 条件启动
    launch_list = [delay_drivers, delay_algorithms]

    if launch_params['navigation']:
        launch_list.append(navigation_node)

    # 5. 添加命名空间
    push_ns = PushRosNamespace(launch_params['namespace'])

    return LaunchDescription([push_ns] + launch_list)
```

### 13.9 进程内通信 vs 进程间通信

| 模式 | 适用场景 | 优点 | 缺点 |
|------|----------|------|------|
| ComposableNodeContainer | 相机-检测器等高频数据 | 零拷贝、低延迟 | 隔离性差 |
| 独立 Node | 跨进程通信、低频数据 | 隔离性好 | 有复制开销 |

**建议**：
- 图像/点云等高频数据使用组件容器
- 控制命令、低频数据使用独立节点

## 13. 构建命令

```bash
# 编译所有包
colcon build --symlink-install

# 单独编译
colcon build --packages-select <package_name>

# 运行测试
colcon test
colcon test-result --verbose
```

### 13.1 系统依赖

部分驱动使用系统包，需要提前安装：

```bash
# RealSense 相机驱动
sudo apt install ros-humble-realsense2-camera ros-humble-librealsense2
```

## 14. 依赖关系

```
inspection_interface (无依赖)
         ↑
inspection_bringup, inspection_supervisor
         ↑
驱动层: agv_driver, arm_driver, hikvision_driver
         ↑
控制层: arm_controller (依赖 arm_driver + MoveIt2)
         ↑
算法层: defect_detector
         ↑
task_coordinator (依赖所有)
```

## 15. 关键技术细节

### 15.1 到位门控

```
agv_ready = connected && arrived && stopped && (error_code == "OK")
```

## 16. 常见问题

### agv_driver
1. 启动时报网卡错误：检查 `elfin_ethernet_name` 配置
2. 目标点被拒绝：检查坐标系是否为 `map`

### arm_driver
1. 机械臂不动：先调用 `enable`
2. 关节跳变：检查 `count_zeros` 标定值

### hikvision_driver
1. 找不到相机：检查 SN 或 device_index
2. 图像异常：调整曝光、增益参数
