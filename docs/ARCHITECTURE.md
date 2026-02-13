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
│ agv_driver   │  │ arm_controller  │  │task_coordinator│
│ arm_driver   │  │  (MoveIt2)      │  │  (状态机)       │
│ hikvision_   │  │                 │  │                 │
│   driver     │  └─────────────────┘  └─────────────────┘
│ realsense_   │         │                    │
│   driver     │         ▼                    │
│               │  ┌─────────────────┐        │
│               │  │    算法层       │        │
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

| 包名 | 职责 | 命名空间 |
|------|------|----------|
| `agv_driver` | AGV 底盘 TCP 驱动 | `/inspection/agv` |
| `arm_driver` | 机械臂 EtherCAT 驱动 | `/inspection/arm` |
| `hikvision_driver` | 海康工业相机驱动 | `/inspection/hikvision` |
| `elfin_ethercat_driver` | EtherCAT 底层驱动 | (库) |
| `soem_ros2` | SOEM 协议栈 | (库) |

### 4.2 控制层 (Control)

| 包名 | 职责 | 命名空间 |
|------|------|----------|
| `arm_controller` | MoveIt2 运动控制 | `/inspection/arm_control` |

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

### 7.4 task_coordinator

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

### 7.5 pose_detector

**订阅**：
- `/inspection/realsense/depth/color/points` (PointCloud2)

**发布**：
- `~/detected_pose` (PoseStamped) - 检测到位姿
- `~/confidence` (Float32) - 置信度

**服务**：
- `~/detect` - 触发检测

### 7.6 path_planner

**订阅**：
- `/inspection/perception/detected_pose` - 工件位姿
- `/inspection/agv/current_pose` - AGV 当前位姿

**发布**：
- `~/path` (PoseArray) - 规划路径

**服务**：
- `~/optimize` - 触发规划

### 7.7 defect_detector

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

## 13. 构建命令

```bash
# 编译所有包
colcon build --symlink-install

# 跳过有问题的包
colcon build --packages-skip realsense2_camera

# 单独编译
colcon build --packages-select <package_name>

# 运行测试
colcon test --packages-skip realsense2_camera
colcon test-result --verbose
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
