# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## 项目概述

**复合移动机器人抓取系统**（ROS2 Humble）

- AGV（仙宫智能）+ 协作机械臂（大族 E05 / Elfin5）+ 手眼相机（RealSense，装机械臂末端）+ 气动二指夹爪
- 算力平台：Jetson Orin AGX 64G
- 技术方案：AGV 到抓取点位 → 机械臂到观察位 → RealSense 取 RGBD → 感知输出抓取位姿 → MoveIt2 规划 → 夹爪抓取 → 放置

> **历史说明**：仓库名 `inspection-robot` 和大部分 `inspection_*` 包名来自早期"大型工件视觉检测/巡检"场景。课题已调整为"复合机器人抓取"，包名保留以避免大范围破坏性重命名；新增能力使用功能命名（`grasp_perception` / `gripper_driver` 等）。

## 常用命令

### 构建
```bash
# 编译所有包
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 单独编译某个包
colcon build --packages-select <package_name>
```

### 测试
```bash
colcon test
colcon test-result --verbose
```

### 运行
```bash
# 必须先 source 环境
source /opt/ros/humble/setup.bash
source ~/inspection-robot/install/setup.bash

# 启动驱动（相机/机械臂/AGV/DIO）
ros2 launch inspection_bringup drivers.launch.py

# 启动完整抓取系统
ros2 launch inspection_bringup system.launch.py
```

## 代码规范

**重要：私有成员变量必须以下划线 `_` 开头，禁止使用 `_` 结尾**

```cpp
// ✅ 正确
int _count;
rclcpp::Publisher::SharedPtr _pub;

// ❌ 错误
int count_;
rclcpp::Publisher::SharedPtr pub_;
```

## Launch 文件规范

参考 rm_bringup 和 radar_bringup 的设计：

1. **参数文件驱动**：使用 YAML 配置文件，不在代码中硬编码
2. **组件容器**：图像类节点使用 `ComposableNodeContainer` 实现进程内通信
3. **延迟启动**：使用 `TimerAction` 避免竞争条件
4. **条件编译**：根据参数决定启动哪些节点
5. **命名空间**：使用 `PushRosNamespace` 统一管理

详细规范见 `docs/ARCHITECTURE.md` 第 13 节。

## 项目架构

```
src/
├── elfin_sdk/                # Elfin 机械臂底层驱动
│   ├── elfin_ethercat_driver/ # EtherCAT 驱动
│   ├── soem_ros2/            # SOEM 协议栈
│   └── elfin_core/           # Elfin 上层功能包（MoveIt2 配置/消息/API）
│       ├── elfin5_ros2_moveit2/  # MoveIt2 配置
│       ├── elfin_basic_api/      # 厂商 API（ROS1 移植）
│       └── elfin_robot_msgs/     # 消息定义
├── arm_driver/               # 机械臂驱动 (调用 elfin_sdk)
├── arm_controller/           # MoveIt2 运动控制
├── agv_driver/               # AGV 底盘驱动 (TCP)
├── realsense_driver/         # RealSense 深度相机（核心传感器，eye-in-hand）
├── dio_driver/               # 工控机板载 DIO 驱动（气动夹爪 DO 底层）
├── gripper_driver/           # 气动二指夹爪语义层（open/close）[规划新增]
├── grasp_perception/         # RGBD → 目标定位 + 抓取位姿 [规划新增]
├── task_coordinator/         # 抓取任务状态机编排
├── inspection_interface/     # 消息/服务定义（保留历史命名）
├── inspection_gateway/       # Web gateway (FastAPI REST/WS, HMI <-> ROS2 桥接)
├── inspection_bringup/       # 启动文件
├── inspection_supervisor/    # 系统健康监控
├── inspection_sim/           # 无硬件端到端联调
├── elfin_description/        # Elfin5 URDF 模型（独立顶层包）
├── hikvision_driver/         # 海康工业相机驱动 [抓取场景已不再是主路径，保留备用]
└── defect_detector/          # 图像缺陷检测 [巡检遗留，将在抓取场景中弃用]
```

### 层级关系
- 驱动层：`elfin_sdk`（`elfin_ethercat_driver` / `soem_ros2`）、`arm_driver`、`agv_driver`、`realsense_driver`、`dio_driver`、`hikvision_driver`（备用）
- 控制层：`arm_controller`（MoveIt2）、`gripper_driver`（语义封装 DIO）
- 算法层：`grasp_perception`（目标定位 + 抓取位姿）；`defect_detector`（将弃用）
- 协调层：`task_coordinator`（抓取 pipeline 状态机）
- 基础设施：`inspection_interface`、`inspection_bringup`、`inspection_supervisor`、`inspection_sim`
- 对外桥接：`inspection_gateway`（FastAPI REST/WS server，运行在机器人端，前端源码在独立 `inspection-site` 仓库）

### 注意事项
- 官方 elfin_ros_control 未移植（使用 topic 通信代替 ros2_control）
- elfin5 的 URDF 在 elfin_description/（独立顶层包，非 elfin_core 子目录）
- realsense_driver 是适配层，使用系统包 ros-humble-realsense2-camera
- RealSense **安装在机械臂末端（eye-in-hand）**，手眼标定是精度上限的关键
- 气动二指夹爪通过 `dio_driver` 的 DO 线控制（研华 AGX TCA9539 I2C GPIO 扩展），上层由 `gripper_driver` 封装 `open/close` 语义

### 系统依赖
部分驱动使用系统包，需要提前安装：
```bash
# RealSense 相机驱动
sudo apt install ros-humble-realsense2-camera ros-humble-librealsense2

# MoveIt2（arm_controller 需要）
sudo apt install ros-humble-moveit

# Python 感知栈（grasp_perception 规划使用）
pip install ultralytics opencv-python open3d
```

## 分包架构约定（避免代码堆在 Node 回调里）

本仓库的 ROS2 包很多，最容易退化成"所有逻辑都写在 node 的构造函数/回调里"。为了长期可维护，约定如下：

1. **Node 只做 IO 与调度**：参数、pub/sub/srv、timer、TF、日志；不做协议细节/算法细节/大量业务分支。
2. **核心逻辑放到 Core/Service 类**：可单测、尽量无 ROS 依赖（或极少 ROS 依赖）。
3. **硬件/协议放到 Adapter/Transport**：对外暴露语义 API；禁止在上层散落 cmd 号/端口/JSON 字段/DO 线号/相机 SDK 调用。
4. **跨包接口不要用 `~/`**：公共 topic/service 用相对名 `topic`，通过 launch 的 `namespace` 固定前缀；`~/` 仅用于节点私有调试接口。

每个包的"职责边界/数据流/推荐文件结构"请看对应的：

- `src/<package_name>/AGENTS.md`（若无则参考 `src/<package_name>/CLAUDE.md`）

## 抓取任务 pipeline（由 `task_coordinator` 编排）

```
IDLE → NAV_TO_PICK → ARM_TO_OBSERVE → CAPTURE_RGBD → PERCEIVE
  → TRANSFORM_IK → PRE_GRASP → GRASP → CLOSE_GRIPPER → LIFT
  → NAV_TO_PLACE（可选）→ PLACE → OPEN_GRIPPER → HOME → IDLE

任一阶段失败 → RECOVERY（重拍/重规划 N 次 → 超限终止并发事件）
```

详细状态机与接口见 `src/task_coordinator/CLAUDE.md` 和 `docs/ARCHITECTURE.md`。

## TODO 维护（必须）

- 仓库根 `TODO.md` 是"还要做什么"的单一事实来源；新增/变更任务必须写这里
- 完成任务必须勾选，并在提交信息里注明验证方式（真机/仿真/回放/单测）
- 修改 public ROS API 或端到端数据流时，必须同步更新：
  - 相关包的 `src/<package>/AGENTS.md` / `README.md`
  - `docs/WORKSPACE_OVERVIEW.md` / `docs/ARCHITECTURE.md`（按需）
  - `TODO.md`

## Git Commit 规范

你是一个专业的 Git 提交信息生成助手。请严格按照以下规范生成 commit 信息。

### 基本格式
```
<emoji> <type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

### Emoji + Type 对照表

- ✨ `feat`: 新增功能
- 🐛 `fix`: 修复 bug
- 📝 `docs`: 文档更新
- 💄 `style`: 代码格式调整(不影响功能)
- ♻️ `refactor`: 代码重构(不增加功能,不修复bug)
- ⚡️ `perf`: 性能优化
- ✅ `test`: 测试相关
- 🔧 `chore`: 构建/工具/依赖更新
- 🔨 `build`: 构建系统修改
- 👷 `ci`: CI/CD 配置修改
- 💥 `BREAKING CHANGE`: 破坏性变更(使用感叹号!)

### 规则说明

1. **类型(必填)**: 使用上述 type 之一
2. **范围(可选)**: 用圆括号标注影响范围,如 `(api)` `(user)`
3. **描述(必填)**: 简短说明变更内容,建议不超过50字
4. **破坏性变更**:
   - 在类型后加 `!` 或在 footer 中使用 `BREAKING CHANGE:`
   - 必须说明影响和迁移方法
5. **正文(可选)**: 详细说明变更原因、内容
6. **页脚(可选)**: 关联 issue 或说明破坏性变更

### 示例

#### 示例1: 基础功能
```
✨ feat: 增加用户搜索功能
```

#### 示例2: 带范围和正文
```
✨ feat(grasp): 增加 YOLOv8 目标检测

1. 支持多类工业件
2. 输出 bbox + depth 合成的候选抓取点
3. 对接 task_coordinator PERCEIVE 阶段
```

#### 示例3: 破坏性变更
```
🔨 build!: 升级依赖库版本

BREAKING CHANGE: 需要重新执行 pip install,Python 版本需 >=3.10
```

#### 示例4: 关联 issue
```
🐛 fix(gripper): 修复 DO 线号映射错误

Closes: #123
```

#### 示例5: 完整格式
```
✨ feat(grasp_perception): 新增 6D 位姿估计模块

功能详情:
1. 集成 FoundationPose 模型
2. 支持 CAD 参考的已知工业件
3. 输出 grasp_pose (camera_frame)

注意事项: 需要 CUDA 12 + 预先标注抓取点

BREAKING CHANGE: PerceiveGrasp.srv 新增 object_class 字段

Reviewed-by: 张三
Closes: #234, #235
```

### 生成要求

- 所有描述使用中文
- emoji 必须放在最前面
- 描述要简洁明确,一句话说清楚做了什么
- 如有破坏性变更,必须明确标注并说明影响
- 优先使用常用类型: feat, fix, docs, refactor, perf
- **提交信息保持简洁，严禁添加任何自动生成标记**：
  - 禁止添加 "🤖 Generated with [Codex](https://Codex.com/Codex)"
  - 禁止添加 "Co-Authored-By: Codex <noreply@anthropic.com>"
  - 禁止添加任何其他AI工具生成的标记
  - 只包含人为编写的提交内容
