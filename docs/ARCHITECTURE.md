# inspection-robot 系统架构文档

> 仓库名 `inspection-robot` 是历史命名（巡检场景遗留）；当前课题方向是**复合移动机器人抓取**。详情见 `README.md` / `CLAUDE.md`。

## 1. 项目概述

基于移动协作机械臂的**复合机器人抓取系统**（ROS2 Humble）。

- **硬件组成**：
  - 仙宫智能 AGV（移动平台，SLAM 导航）
  - 大族 E05 / Elfin5 协作机械臂（6 自由度）
  - Intel RealSense D435 深度相机（**eye-in-hand**，装机械臂末端）
  - 气动二指夹爪（通过工控机 DIO 驱动电磁阀）
  - 海康工业相机（原巡检方向遗留，当前默认不启用）
- **算力平台**：Jetson Orin AGX 64G
- **技术方案**：AGV 到抓取点位 → 机械臂到观察位 → RealSense 取 RGBD → 感知输出抓取位姿（YOLOv8+深度 → 已知姿态查表；升级路径：FoundationPose 6D 位姿）→ MoveIt2 + Pilz LIN 规划 → 夹爪抓取 → 放置

## 2. 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 (Jammy) |
| ROS 版本 | ROS 2 Humble |
| 编译器 | GCC 11+ (C++17) |
| Python | 3.10+ |
| 关键依赖 | `ros-humble-realsense2-camera`, `ros-humble-moveit`, `ros-humble-pilz-industrial-motion-planner` |
| Python 包 | `ultralytics`, `opencv-python`, `open3d` |

## 3. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                     前端 (inspection-site)                        │
│                  React HMI (Operator)                             │
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
│                    inspection_interface                           │
│              消息/服务定义 (对齐网关语义)                          │
└────────────────────────────┬────────────────────────────────────┘
                             │
   ┌─────────────────────┼─────────────────────┐
   ▼                     ▼                     ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────────┐
│   驱动层      │   │    控制层    │   │    协调层        │
│               │   │              │   │                  │
│ elfin_sdk    │   │ arm_controller │   │ task_coordinator │
│ arm_driver   │   │  (MoveIt2 +    │   │  (抓取状态机)    │
│ agv_driver   │   │   Pilz LIN)    │   │                  │
│ realsense_   │   │ gripper_driver │   │                  │
│   driver     │   │  (open/close)  │   │                  │
│ dio_driver   │   └──────┬─────────┘   └──────┬───────────┘
└──────┬───────┘          │                    │
       │                  ▼                    │
       │           ┌──────────────┐            │
       │           │   算法层     │            │
       │           │              │            │
       │           │ grasp_       │◄───────────┘
       │           │  perception  │
       │           │              │
       └──────────►│ (RGBD→pose)  │
                   └──────────────┘

┌───────────────────────────────────────────────────────┐
│              inspection_bringup / inspection_supervisor│
│                  启动管理 / 系统监控                  │
└───────────────────────────────────────────────────────┘
```

说明：
- `inspection_gateway` 在本仓库中以 ROS2 包形式存在（部署在 AGX）：`src/inspection_gateway/`
- 前端不直接连接 ROS2，所有对外 API 以 `inspection_gateway/api/models.py`（Pydantic v2）为准

## 4. 功能包索引

### 4.1 驱动层 (Drivers)

| 包名 | 职责 | 备注 |
|------|------|------|
| `elfin_sdk` | Elfin 机械臂底层驱动 | 包含 elfin_ethercat_driver, soem_ros2, elfin_core |
| `arm_driver` | 机械臂 EtherCAT 驱动 | 调用 elfin_sdk，`/inspection/arm` |
| `agv_driver` | AGV 底盘 TCP 驱动 | `/inspection/agv` |
| `realsense_driver` | RealSense 相机适配层（eye-in-hand） | 使用系统包 `ros-humble-realsense2-camera` |
| `dio_driver` | 工控机板载 DIO（TCA9539 GPIO 扩展） | `/inspection/dio`，夹爪 DO 的底层 |
| `hikvision_driver` | 海康工业相机驱动（巡检遗留） | 抓取场景默认不启用 |

### 4.2 控制层 (Control)

| 包名 | 职责 | 备注 |
|------|------|------|
| `arm_controller` | MoveIt2 运动控制 | 运行时依赖 `elfin5_ros2_moveit2`；需启用 Pilz LIN plugin 做 PRE_GRASP→GRASP 直线段 |
| `gripper_driver` **[新增规划]** | 气动二指夹爪语义层（open/close） | 底层调 `dio_driver/set_output`；DO 线号通过 yaml 配置 |

### 4.3 算法层 (Algo)

| 包名 | 职责 | 命名空间 |
|------|------|----------|
| `grasp_perception` **[新增规划]** | RGBD → 目标定位 + 抓取位姿 | `/inspection/grasp`；路径 A：YOLOv8+深度+姿态查表；路径 B：FoundationPose 6D |
| `defect_detector` | 图像缺陷检测（巡检遗留） | 抓取场景不使用，待清理 |

### 4.4 协调层 (Coordination)

| 包名 | 职责 | 命名空间 |
|------|------|----------|
| `task_coordinator` | 抓取任务状态机 | `/inspection` |

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
3. **设备厂商层（最低）**：AGV TCP API / RealSense SDK / 相机 SDK / GPIO（仅驱动内部使用）

## 6. 抓取任务状态机

```
IDLE ──▶ NAV_TO_PICK ──▶ ARM_TO_OBSERVE ──▶ CAPTURE_RGBD ──▶ PERCEIVE
  ▲                                                                │
  │                                                                ▼
  │                                                         TRANSFORM_IK
  │                                                                │
  │                                                                ▼
  │                                                          PRE_GRASP
  │                                                                │
  │                                                                ▼
  │                                                            GRASP
  │                                                                │
  │                                                                ▼
  │                                                       CLOSE_GRIPPER
  │                                                                │
  │                                                                ▼
  │                                                             LIFT
  │                                                                │
  │                                                                ▼
  │                                                        NAV_TO_PLACE (可选)
  │                                                                │
  │                                                                ▼
  │                                                             PLACE
  │                                                                │
  │                                                                ▼
  │                                                        OPEN_GRIPPER
  │                                                                │
  │                                                                ▼
  │                                                             HOME
  │                                                                │
  └────────────────────────────────────────────────────────────────┘

任一阶段失败 → RECOVERY（重拍/重规划 N 次 → 超限终止并发事件）
任一阶段可被 PAUSE / STOP 中断
```

| 状态 | 说明 | 成功条件 |
|------|------|---------|
| IDLE | 空闲，等待任务 | — |
| NAV_TO_PICK | AGV 导航到抓取点位（yaml 配置） | `agv_status.arrived && stopped && error_code=="OK"` |
| ARM_TO_OBSERVE | 机械臂到观察位（让末端相机完整看到工作台） | `arm_status.arrived` |
| CAPTURE_RGBD | RealSense 同步抓取 RGB + Depth + 内参 | 三者时间戳差 < 50ms |
| PERCEIVE | `grasp_perception/perceive_grasp` 输出抓取候选 | `len(candidates) >= 1` |
| TRANSFORM_IK | 候选变换到 `arm_base` + MoveIt 可达性筛选 | 至少 1 个候选 IK 通过且无碰撞 |
| PRE_GRASP | Pilz LIN 到抓取点上方 ~10cm | `motion_status == done` |
| GRASP | Pilz LIN 直线下压到抓取点 | `motion_status == done` |
| CLOSE_GRIPPER | `gripper/close` 闭合电磁阀 | `gripper_status.is_closed` 或固定等待 300ms |
| LIFT | 上升 ~10cm | `motion_status == done` |
| NAV_TO_PLACE | （可选）AGV 到放置点 | 同 NAV_TO_PICK |
| PLACE | 到放置点上方 → 下压 → OPEN_GRIPPER | 序列完成 |
| HOME | 机械臂收回 home 关节角 | `motion_status == done` |
| PAUSED / STOPPED / FAILED | 控制面干预或异常 | — |

## 7. ROS 接口

### 7.1 agv_driver

**命名空间**：`/inspection/agv`

**订阅**：
- `goal_pose` (PoseStamped) - 导航目标（map 坐标系）
- `goal_station` (String) - 按 AGV 站点名导航
- `cmd_vel` (Twist) - 手动速度指令

**发布**：
- `status` (`inspection_interface/AgvStatus`)
- `current_pose` (PoseStamped)
- `odom` (Odometry)

**服务**：
- `get_nav_map` (`inspection_interface/srv/GetNavMap`) - 导航底图，供网关对外实现 GetNavMap

### 7.2 arm_driver

**命名空间**：`/inspection/arm`

**订阅**：`joint_cmd` (JointState)
**发布**：`/joint_states` (JointState)、`status` (`inspection_interface/ArmStatus`)
**服务**：`enable` / `disable` / `clear_fault` / `stop` (`std_srvs/Trigger`)

### 7.3 realsense_driver

**命名空间**：`/inspection/realsense/d435`

**发布（常用）**：
- `color/image_raw` (Image)
- `depth/image_rect_raw` (Image)
- `aligned_depth_to_color/image_raw` (Image) ← 抓取主用
- `color/camera_info` (CameraInfo) ← 内参
- `depth/color/points` (PointCloud2) ← 可选
- TF：`camera_link` / `camera_color_optical_frame` / `camera_depth_optical_frame`

### 7.4 dio_driver

**命名空间**：`/inspection/dio`

**发布**：`status` (`inspection_interface/DioStatus`) @ 20Hz
**服务**：`set_output` (`inspection_interface/srv/SetDioOutput`)

### 7.5 gripper_driver（新增）

**命名空间**：`/inspection/gripper`

**服务**：
- `open` (`std_srvs/srv/Trigger`)
- `close` (`std_srvs/srv/Trigger`)

**发布**：`status` (`inspection_interface/GripperStatus`)

**底层**：调用 `/inspection/dio/set_output`，DO 线号在 yaml 里配置。

### 7.6 grasp_perception（新增）

**命名空间**：`/inspection/grasp`

**订阅**：
- `/inspection/realsense/d435/color/image_raw`
- `/inspection/realsense/d435/aligned_depth_to_color/image_raw`
- `/inspection/realsense/d435/color/camera_info`

**服务**：
- `perceive_grasp` (`inspection_interface/srv/PerceiveGrasp`)
  - request: `{string object_class (可选)}`
  - response: `{bool success; string error; GraspPose[] candidates; string frame_id}`

**发布（可视化/调试）**：
- `candidates` (PoseArray) - 抓取候选（frame_id = `camera_color_optical_frame`）
- `annotated_image` (Image) - 带检测框的彩色图

### 7.7 arm_controller

**命名空间**：`/inspection/arm_control`

**订阅**：
- `cart_goal` (PoseStamped)
- `joint_goal` (JointState)
- `velocity_scaling` (Float64)

**服务**：
- `move_to_pose` (`inspection_interface/srv/MoveToPose`) - 支持 `planner_id` 参数（OMPL/PILZ_LIN）
- `move_to_joints` (`inspection_interface/srv/MoveToJoints`)

**发布**：
- `motion_status` (String) - planning/executing/done/failed
- `trajectory_progress` (Float64)

### 7.8 task_coordinator

**命名空间**：`/inspection`

**发布**：`state` (`inspection_interface/SystemState`)

**订阅**：
- `agv/status`、`arm/status`、`gripper/status`
- `realsense/d435/color/image_raw` / `aligned_depth_to_color/image_raw`（供抓拍/可视化）

**服务（对外）**：
- `start` (`inspection_interface/srv/StartGrasp`) - 替代原 `StartInspection`
- `stop` / `pause` / `resume` / `get_status`

**服务客户端**：
- `arm_control/move_to_pose` / `move_to_joints`
- `grasp/perceive_grasp`
- `gripper/open` / `gripper/close`
- `agv/` 相关

**配置**：
- `grasp_tasks_file` 参数指向 `grasp_tasks.yaml`（定义 pick/place 点位、观察位、目标类别）

## 8. 消息/服务定义

不在此处复制字段表，以实现为准：
- 机器人内部接口：`src/inspection_interface/msg`、`src/inspection_interface/srv`
- 对外 API 模型：`src/inspection_gateway/inspection_gateway/api/models.py`

新增（规划中，见 `TODO.md`）：
- `srv/PerceiveGrasp.srv`
- `srv/StartGrasp.srv`（替代 `StartInspection.srv` 或新增）
- `msg/GraspPose.msg`
- `msg/GripperStatus.msg`

## 9. TF 树

```
map (AGV 全局坐标系)                        ← agv_driver 发布 map -> base_link
 └─ base_link (AGV 底盘)
      └─ arm_base (机械臂基座)                ← ⚠ 静态标定，当前缺失，需补充
           └─ elfin_link1..6 (URDF 链)
                └─ tool0 (末端)               ← URDF 正解
                     ├─ gripper_tip           ← 静态 TF（气动夹爪末端 offset）
                     └─ camera_link           ← 手眼标定（tool0 → camera_link，yaml 驱动）
                          └─ camera_color_optical_frame ← RealSense 发布
```

关键标定（静态 TF，由 `inspection_bringup` 发布）：
- `base_link → arm_base`：机械臂安装标定
- `tool0 → camera_link`：手眼标定（推荐 `easy_handeye2` + ChArUco）
- `tool0 → gripper_tip`：夹爪机械 offset

## 10. 数据流

### 10.1 初始化阶段
```
启动 inspection_bringup → 加载配置 → 驱动连接 → TF 发布 → 网关起 REST/WS
```

### 10.2 抓取执行（单次）
```
user → gateway (POST /tasks) → task_coordinator (StartGrasp)
 ├─ NAV_TO_PICK:       task_coordinator → agv/goal_pose → agv_driver
 │                     wait agv_status.arrived && stopped
 ├─ ARM_TO_OBSERVE:    task_coordinator → arm_control/move_to_joints → arm_controller
 ├─ CAPTURE_RGBD:      realsense_driver → task_coordinator (缓存最新 RGBD)
 ├─ PERCEIVE:          task_coordinator → grasp/perceive_grasp → grasp_perception
 │                     grasp_perception ← realsense_driver (订阅 color/depth/info)
 ├─ TRANSFORM_IK:      task_coordinator: TF camera → arm_base + arm_control/move_to_pose 可达性
 ├─ PRE_GRASP:         task_coordinator → arm_control/move_to_pose (PILZ_LIN) → arm_controller → arm_driver
 ├─ GRASP:             同上 (PILZ_LIN 直线下压)
 ├─ CLOSE_GRIPPER:     task_coordinator → gripper/close → gripper_driver → dio_driver → 电磁阀
 ├─ LIFT:              task_coordinator → arm_control/move_to_pose (PILZ_LIN)
 └─ NAV_TO_PLACE → PLACE → OPEN_GRIPPER → HOME → IDLE

事件与状态：task_coordinator 全程发布 /inspection/state → gateway 推 WebSocket → 前端显示
```

## 11. 启动方式

```bash
# 驱动层（相机 + 机械臂 + AGV + DIO）
ros2 launch inspection_bringup drivers.launch.py

# 完整系统（drivers + 控制 + 感知 + 夹爪 + 编排 + 网关）
ros2 launch inspection_bringup system.launch.py

# 单包启动
ros2 launch agv_driver agv_driver.launch.py
ros2 launch arm_driver arm_driver.launch.py
ros2 launch realsense_driver realsense.launch.py
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
- 命名空间：`/inspection/*`（历史保留）
- 对外 REST/WS API：以 `inspection_gateway/api/models.py` 为准

## 13. Launch 文件规范（参考 rm_bringup & radar_bringup）

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
│   ├── drivers.launch.py         # 驱动层（相机/臂/AGV/DIO + 静态 TF）
│   ├── camera_only.launch.py     # 仅相机（调试用）
│   └── ...
├── config/
│   ├── inspection.yaml           # 全局参数
│   ├── realsense.yaml            # RealSense 参数
│   ├── mount_calib.yaml          # base_link → arm_base 静态标定
│   ├── handeye_calib.yaml        # tool0 → camera_link 手眼标定
│   ├── gripper_offset.yaml       # tool0 → gripper_tip offset
│   └── grasp_tasks.yaml          # pick/place 点位 + 观察位 + 类别
└── package.xml
```

### 13.3 参数文件示例

```yaml
# config/inspection.yaml（抓取场景）
namespace: "inspection"
use_agv: true          # 可切为 false 跑 arm-only demo
use_hikvision: false   # 巡检场景专用相机，抓取不启用
agv:
  ip: "192.168.1.10"
  bootstrap_timeout_ms: 60000
arm:
  ethernet_name: "enp3s0"
  state_publish_rate_hz: 100.0
realsense:
  enable_pointcloud: true
  align_depth: true
grasp_perception:
  model_path: "package://grasp_perception/models/yolov8n_parts.pt"
  score_threshold: 0.5
gripper:
  do_line_close: 0
  do_line_open: 1
```

### 13.4 组件容器示例

```python
# 将相机和感知放到同一容器，实现零拷贝
def get_perception_container():
    return ComposableNodeContainer(
        name='perception_container',
        namespace='inspection',
        package='rclcpp_components',
        executable='component_container_isolated',
        arguments=['--use_multi_threaded_executor'],
        composable_node_descriptions=[
            # RealSense 可通过 intra_process_comms 把 color/depth 直接传给 grasp_perception
            ComposableNode(
                package='grasp_perception',
                plugin='grasp_perception::GraspPerceptionNode',
                name='grasp_perception',
                namespace='inspection/grasp',
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
from launch.actions import TimerAction

# 驱动先起，2 秒后起感知/控制
delay_perception = TimerAction(period=2.0, actions=[perception_container])
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
```

### 13.8 完整示例结构

```python
def generate_launch_description():
    # 1. 加载参数
    launch_params = yaml.safe_load(open(...))

    # 2. 定义节点（使用函数封装）
    drivers = get_drivers_container()
    perception = get_perception_container()
    controller = get_arm_controller_node()
    coordinator = get_task_coordinator_node()

    # 3. 延迟启动
    delay_drivers = TimerAction(period=1.0, actions=[drivers])
    delay_perception = TimerAction(period=2.0, actions=[perception])
    delay_coord = TimerAction(period=3.0, actions=[coordinator])

    # 4. 条件启动
    launch_list = [delay_drivers, delay_perception, controller, delay_coord]
    if launch_params['use_agv']:
        launch_list.append(agv_node)
    if launch_params['use_hikvision']:
        launch_list.append(hikvision_container)

    # 5. 添加命名空间
    push_ns = PushRosNamespace(launch_params['namespace'])

    return LaunchDescription([push_ns] + launch_list)
```

### 13.9 进程内通信 vs 进程间通信

| 模式 | 适用场景 | 优点 | 缺点 |
|------|----------|------|------|
| ComposableNodeContainer | 相机-感知等高频数据 | 零拷贝、低延迟 | 隔离性差 |
| 独立 Node | 跨进程通信、低频数据 | 隔离性好 | 有复制开销 |

**建议**：
- RealSense → grasp_perception 使用组件容器（RGBD 高频）
- 控制命令、状态推送使用独立节点

## 14. 构建命令

```bash
# 编译所有包
colcon build --symlink-install

# 单独编译
colcon build --packages-select <package_name>

# 运行测试
colcon test
colcon test-result --verbose
```

### 14.1 系统依赖

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-librealsense2
sudo apt install ros-humble-moveit ros-humble-pilz-industrial-motion-planner
pip install ultralytics opencv-python open3d
```

## 15. 依赖关系

```
inspection_interface (无依赖)
         ↑
inspection_bringup, inspection_supervisor
         ↑
驱动层: agv_driver, arm_driver, realsense_driver, dio_driver
         ↑
控制层: arm_controller (依赖 arm_driver + MoveIt2)
        gripper_driver (依赖 dio_driver)
         ↑
算法层: grasp_perception (依赖 realsense_driver + TF)
         ↑
task_coordinator (依赖所有)
```

## 16. 关键技术细节

### 16.1 到位门控

```
agv_ready = connected && arrived && stopped && (error_code == "OK")
arm_ready = connected && enabled && arrived && (error_code == "OK")
gripper_ready = (is_closed == expected) && !moving
```

### 16.2 坐标变换（抓取位姿）

感知在 `camera_color_optical_frame` 输出；`task_coordinator` 变换到 `arm_base`：

```
T_armbase_grasp = T_armbase_base * T_base_tool0 * T_tool0_cam * T_cam_grasp
```

其中 `T_tool0_cam` 来自手眼标定 yaml；`T_base_tool0` 由 TF tree（URDF 正解）；`T_armbase_base` 是静态 TF 的逆。

### 16.3 Pilz LIN 用于 PRE_GRASP → GRASP

MoveIt 默认 OMPL 走弧线，会撞工作台。PRE_GRASP → GRASP 段必须用 Pilz LIN（直线插补）。调用时在 `move_to_pose` 里传 `planner_id: "PILZ_LIN"`。

## 17. 常见问题

### agv_driver
1. 启动时报网卡错误：检查 `elfin_ethernet_name` 配置
2. 目标点被拒绝：检查坐标系是否为 `map`

### arm_driver
1. 机械臂不动：先调用 `enable`
2. 关节跳变：检查 `count_zeros` 标定值

### realsense_driver
1. 找不到相机：检查 USB3.0（不是 2.0）；`lsusb` 能看到 Intel RealSense
2. 深度空洞多：调整工作距离（D435 推荐 0.2–2m）；检查环境光

### grasp_perception（规划）
1. 感知无目标：检查 YOLOv8 权重路径、score_threshold；确认观察位下工件在视野内
2. 候选都 IK 失败：观察位选得太偏；换个观察位再试

### gripper_driver（规划）
1. 夹爪不动：检查 DO 线号 yaml；检查气压
2. 夹到一半松开：确认电磁阀类型（单动 vs 双动）、`is_closed` 反馈是否来自 DI
