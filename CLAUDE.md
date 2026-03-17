# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于移动协作机械臂的大型工件视觉检测系统 (ROS2 Humble)

- AGV (仙宫智能) + 机械臂 (大族E05) + 深度相机 (RealSense) + 工业相机 (海康)
- 技术方案：AGV站位 + 机械臂逆解的联合优化

## 常用命令

### 构建
```bash
# 编译所有包（跳过有问题的 realsense2_camera）
source /opt/ros/humble/setup.bash
colcon build --packages-skip realsense2_camera

# 单独编译某个包
colcon build --packages-select <package_name>
```

### 测试
```bash
colcon test --packages-skip realsense2_camera
colcon test-result --verbose
```

### 运行
```bash
# 必须先 source 环境
source /opt/ros/humble/setup.bash
source ~/inspection-robot/install/setup.bash

# 启动驱动（相机）
ros2 launch inspection_bringup drivers.launch.py

# 启动完整系统
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
├── hikvision_driver/         # 海康工业相机驱动
├── pose_detector/            # 6D 位姿检测
├── path_planner/             # AGV+机械臂联合路径规划
├── defect_detector/          # 图像缺陷检测
├── task_coordinator/         # 任务状态机编排
├── inspection_interface/     # 消息/服务定义
├── inspection_gateway/       # Web gateway (FastAPI REST/WS, HMI <-> ROS2 桥接)
├── inspection_bringup/      # 启动文件
├── inspection_supervisor/    # 系统健康监控
├── elfin_description/       # Elfin5 URDF 模型（独立顶层包）
└── realsense_driver/        # RealSense 深度相机
```

### 层级关系
- 驱动层: elfin_sdk (elfin_ethercat_driver, soem_ros2), arm_driver, agv_driver, hikvision_driver
- 控制层: arm_controller (MoveIt2)
- 算法层: pose_detector, path_planner, defect_detector
- 协调层: task_coordinator
- 基础设施: inspection_interface, inspection_bringup, inspection_supervisor
- 对外桥接: inspection_gateway（FastAPI REST/WS server，运行在机器人端，前端源码在独立 inspection-site 仓库）

### 注意事项
- 官方 elfin_ros_control 未移植（使用 topic 通信代替 ros2_control）
- elfin5 的 URDF 在 elfin_description/（独立顶层包，非 elfin_core 子目录）
- realsense_driver 是适配层，使用系统包 ros-humble-realsense2-camera

### 系统依赖
部分驱动使用系统包，需要提前安装：
```bash
# RealSense 相机驱动
sudo apt install ros-humble-realsense2-camera ros-humble-librealsense2
```

## 分包架构约定（避免代码堆在 Node 回调里）

本仓库的 ROS2 包很多，最容易退化成“所有逻辑都写在 node 的构造函数/回调里”。为了长期可维护，约定如下：

1. **Node 只做 IO 与调度**：参数、pub/sub/srv、timer、TF、日志；不做协议细节/算法细节/大量业务分支。
2. **核心逻辑放到 Core/Service 类**：可单测、尽量无 ROS 依赖（或极少 ROS 依赖）。
3. **硬件/协议放到 Adapter/Transport**：对外暴露语义 API；禁止在上层散落 cmd 号/端口/JSON 字段。
4. **跨包接口不要用 `~/`**：公共 topic/service 用相对名 `topic`，通过 launch 的 `namespace` 固定前缀；`~/` 仅用于节点私有调试接口。

每个包的“职责边界/数据流/推荐文件结构”请看对应的：

- `src/<package_name>/CLAUDE.md`

## TODO 维护（必须）

- 仓库根 `TODO.md` 是“还要做什么”的单一事实来源；新增/变更任务必须写这里
- 完成任务必须勾选，并在提交信息里注明验证方式（真机/仿真/回放/单测）
- 修改 public ROS API 或端到端数据流时，必须同步更新：
  - 相关包的 `src/<package>/CLAUDE.md` / `README.md`
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
✨ feat(notice): 增加消息搜索功能

1. 支持按关键词搜索
2. 搜索范围限制在近一个月
3. 支持模糊匹配
```

#### 示例3: 破坏性变更
```
🔨 build!: 升级依赖库版本

BREAKING CHANGE: 需要重新执行 npm install,Node 版本需 >=16
```

#### 示例4: 关联 issue
```
🐛 fix(auth): 修复登录超时问题

Closes: #123
```

#### 示例5: 完整格式
```
✨ feat(payment): 新增支付宝支付方式

功能详情:
1. 集成支付宝 SDK
2. 实现扫码支付流程
3. 添加支付状态回调

注意事项: 需要配置支付宝商户信息

BREAKING CHANGE: 支付接口参数结构调整,需更新调用方代码

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
  - 禁止添加 "🤖 Generated with [Claude Code](https://claude.com/claude-code)"
  - 禁止添加 "Co-Authored-By: Claude <noreply@anthropic.com>"
  - 禁止添加任何其他AI工具生成的标记
  - 只包含人为编写的提交内容
