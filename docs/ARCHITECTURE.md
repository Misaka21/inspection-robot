# inspection-robot 系统架构文档

## 1. 项目概述

基于移动协作机械臂的大型工件视觉检测系统（ROS2 Humble）。

- **硬件组成**：
  - 仙宫智能 AGV（移动平台，SLAM 导航）
  - 大族 E05 协作机械臂（6自由度）
  - Intel RealSense 深度相机（末端安装）
  - 海康工业相机（末端安装）

- **技术方案**：AGV站位 + 机械臂逆解的联合优化

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
│                        上位机 (inspection-api)                   │
│                   inspection_gateway.proto                        │
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
│ arm_driver   │  │ + elfin_core    │  │                 │
│ agv_driver   │  └─────────────────┘  └─────────────────┘
│ hikvision_   │         │                    │
│   driver     │         ▼                    │
│ realsense_   │  ┌─────────────────┐        │
│   (适配层)   │  │    算法层       │        │
│               │  │                 │        │
│               │  │ pose_detector  │◄───────┘
│               │  │ path_planner   │         │
│               │  │ defect_detector│        │
└───────────────┘  └─────────────────┘        │
        │                                        │
        ▼                                        │
┌───────────────────────────────────────────────────────┐
│              inspection_bringup / inspection_supervisor │
│                  启动管理 / 系统监控                   │
└───────────────────────────────────────────────────────┘
```

## 4. 功能包索引

### 4.1 驱动层 (Drivers)

| 包名 | 职责 | 备注 |
|------|------|------|
| `elfin_sdk` | Elfin 机械臂底层驱动 | 包含 elfin_ethercat_driver, soem_ros2 |
| `arm_driver` | 机械臂 EtherCAT 驱动 | 调用 elfin_sdk |
| `agv_driver` | AGV 底盘 TCP 驱动 | `/inspection/agv` |
| `hikvision_driver` | 海康工业相机驱动 | `/inspection/hikvision` |
| `realsense_driver` | RealSense 相机适配层 | 使用系统包 ros-humble-realsense2-camera |

### 4.2 控制层 (Control)

| 包名 | 职责 | 备注 |
|------|------|------|
| `arm_controller` | MoveIt2 运动控制 | 包含 elfin_core (URDF/消息/API) |

### 4.3 算法层 (Algo)

| 包名 | 职责 | 命名空间 |
|------|------|----------|
| `pose_detector` | 6D 位姿检测 | `/inspection/perception` |
| `path_planner` | AGV+机械臂联合路径规划 | `/inspection/planning` |
| `defect_detector` | 图像缺陷检测 | `/inspection/perception` |

### 4.4 协调层 (Coordination)

| 包名 | 职责 | 命名空间 |
|------|------|----------|
| `task_coordinator` | 任务状态机管理 | `/inspection` |

### 4.5 基础设施 (Infra)

| 包名 | 职责 |
|------|------|
| `inspection_interface` | 消息/服务定义 |
| `inspection_bringup` | 启动管理 |
| `inspection_supervisor` | 系统监控 |

## 5. API 优先级

1. **外部契约层（最高）**：`inspection-api/proto/inspection_gateway.proto`
2. **机器人内部编排层**：`inspection_interface`（对齐网关语义）
3. **设备厂商层（最低）**：AGV TCP API / 相机 SDK（仅驱动内部使用）

## 6. 状态机

```
IDLE ──▶ LOCALIZING ──▶ PLANNING ──▶ EXECUTING ──▶ COMPLETED
  ▲          │               │              │
  │          ▼               ▼              ▼
  │       PAUSED ◀────── FAILED ◀──────────┘
  │          │
  └──────────┘ (RESUME/STOP)
```

| 状态 | 说明 |
|------|------|
| PHASE_IDLE | 空闲，等待任务 |
| PHASE_LOCALIZING | 检测工件位姿 |
| PHASE_PLANNING | 规划检测路径 |
| PHASE_EXECUTING | 执行检测 |
| PHASE_PAUSED | 任务暂停 |
| PHASE_COMPLETED | 检测完成 |
| PHASE_FAILED | 检测失败 |
| PHASE_STOPPED | 任务停止 |

## 7. ROS 接口

### 7.1 agv_driver

**订阅**：
- `~/goal_pose` (PoseStamped) - 导航目标（map坐标系）
- `~/cmd_vel` (Twist) - 手动速度指令

**发布**：
- `~/status` (AgvStatus) - 底盘状态
- `~/current_pose` (PoseStamped) - 当前位姿
- `~/odom` (Odometry) - 里程计

**服务**：
- (由 TCP API 提供)

### 7.2 arm_driver

**订阅**：
- `~/joint_cmd` (JointState) - 关节指令

**发布**：
- `/joint_states` (JointState) - 关节状态
- `~/status` (ArmStatus) - 机械臂状态

**服务**：
- `~/enable` - 使能
- `~/disable` - 禁用
- `~/clear_fault` - 清除故障
- `~/stop` - 停止

### 7.3 hikvision_driver

**发布**：
- `~/image_raw` (Image) - 原始图像
- `~/camera_info` (CameraInfo) - 相机参数

**服务**：
- `~/trigger` - 触发拍照

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
- `/inspection/arm_control/cart_goal` (PoseStamped) - 机械臂目标
- `/inspection/arm_control/joint_goal` (JointState) - 机械臂关节目标

**订阅**：
- `/inspection/agv/current_pose` - AGV 当前位姿
- `/joint_states` - 机械臂关节状态

**服务**：
- `~/start` - 启动任务
- `~/stop` - 停止任务
- `~/pause` - 暂停任务
- `~/resume` - 恢复任务
- `~/get_status` - 获取状态

### 7.6 pose_detector

**订阅**：
- `/inspection/realsense/d435/depth/color/points` (PointCloud2)

**发布**：
- `~/detected_pose` (PoseStamped) - 检测到位姿
- `~/confidence` (Float32) - 置信度

**服务**：
- `~/detect` - 触发检测

### 7.7 path_planner

**订阅**：
- `/inspection/perception/detected_pose` - 工件位姿
- `/inspection/agv/current_pose` - AGV 当前位姿

**发布**：
- `~/path` (PoseArray) - 规划路径

**服务**：
- `~/optimize` - 触发规划

### 7.8 defect_detector

**订阅**：
- `/inspection/hikvision/image_raw` (Image)

**发布**：
- `~/result` (DefectInfo) - 检测结果

**服务**：
- `~/detect` - 触发检测

## 8. 消息定义

### SystemState
```
uint8 phase              # 当前阶段
float32 progress_percent # 进度百分比
string current_action    # 当前动作
string error_message     # 错误信息
AgvStatus agv           # AGV 状态
ArmStatus arm           # 机械臂状态
```

### AgvStatus
```
bool connected          # 连接状态
bool arrived           # 到位标志
bool moving            # 运动中
bool stopped           # 停止标志
Pose current_pose      # 当前位姿
float32 battery_percent# 电量
string error_code      # 错误码
```

### ArmStatus
```
bool connected
bool arrived
bool moving
float64[] current_joints   # 当前关节角
float32 manipulability      # 可操作度
string error_code
```

### DefectInfo
```
int32 defect_id
string defect_type
float32 confidence
Point position
Vector3 size
Image image
```

## 9. TF 树

```
map (SLAM全局坐标系)
 └─ base_link (AGV底盘)
      └─ arm_base (机械臂基座)
           └─ link1 → link2 → ... → link6
                └─ tool0 (末端法兰)
                     ├─ realsense_link
                     └─ hikvision_link
```

## 10. 数据流

### 10.1 初始化阶段
```
启动 inspection_bringup → 加载配置 → 驱动连接 → TF 发布
```

### 10.2 工件定位
```
realsense_driver → pose_detector → task_coordinator
    (点云)              (位姿)          (决策)
```

### 10.3 路径规划
```
pose_detector + CAD模型 + agv_pose → path_planner → task_coordinator
```

### 10.4 执行阶段
```
for waypoint in path:
    task_coordinator → agv_driver (goal_pose)
    wait arrived && stopped
    task_coordinator → arm_controller (MoveJ)
    task_coordinator → hikvision_driver (trigger)
    hikvision_driver → defect_detector (image)
    defect_detector → task_coordinator (result)
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
- 使用相对话题名 `~/topic`
- 避免硬编码绝对话题名
- 命名空间：`/inspection/*`

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
│   ├── drivers.launch.py         # 驱动层（相机、AGV、机械臂）
│   ├── algorithms.launch.py      # 算法层（检测、规划）
│   └── ...
├── config/
│   ├── launch_params.yaml       # 全局启动参数
│   ├── node_params/             # 各节点参数
│   │   ├── task_coordinator_params.yaml
│   │   ├── pose_detector_params.yaml
│   │   └── ...
│   └── config.24.home.yaml       # 场景配置文件
└── package.xml
```

### 13.3 参数文件示例

```yaml
# config/launch_params.yaml
rune: false                    # 是否启用打符功能
hero_solver: false             # 是否使用英雄机甲解算
navigation: false              # 是否启用导航
namespace: "inspection"        # 命名空间
video_play: false              # 是否播放视频（调试用）
virtual_serial: false          # 是否使用虚拟串口
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
                package='pose_detector',
                plugin='pose_detector::PoseDetectorNode',
                name='pose_detector',
                namespace='inspection/perception',
                parameters=[node_params],
                extra_arguments=[{'use_intra_process_comms': True}]  # 启用进程内通信
            ),
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
算法层: pose_detector, path_planner, defect_detector
         ↑
task_coordinator (依赖所有)
```

## 15. 关键技术细节

### 15.1 联合优化代价函数

```math
J = w_1\|b-b_{prev}\|^2 + w_2\|q-q_{prev}\|^2 + w_3\frac{1}{m(q)+\epsilon} + w_4E_{view} + w_5E_{limit}
```

其中：
- `b`: AGV 站位 (x, y, yaw)
- `q`: 机械臂关节角
- `m(q)`: 可操作度

### 15.2 位姿链计算

```
T_map_cam* = T_map_base * T_base_arm_base * T_arm_base_tcp * T_tcp_cam
```

### 15.3 到位门控

```
agv_ready = connected && arrived && stopped && (error_code == "OK")
```

## 16. 常见问题

### agv_driver
1. 启动时报网卡错误：检查 `elfin_ethernet_name` 配置
2. 目标点被拒绝：检查坐标系是否为 `map`

### arm_driver
1. 机械臂不动：先调用 `~/enable`
2. 关节跳变：检查 `count_zeros` 标定值

### hikvision_driver
1. 找不到相机：检查 SN 或 device_index
2. 图像异常：调整曝光、增益参数
