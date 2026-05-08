# AGENTS.md

本文件提供 Codex 在本仓库工作时的项目约束。

## 项目概述

**基于移动协作机械臂的大型工件视觉检测/巡检系统**（ROS2 Humble）。

- AGV（仙宫智能）+ 协作机械臂（大族 E05 / Elfin5）+ RealSense 深度相机（末端安装）+ 海康工业相机（末端安装）
- 算力平台：Jetson Orin AGX 64G
- 技术方案：AGV 到巡检站位 → 机械臂到预设观测位 → RealSense 测距并微调工作距 → 海康相机拍照 → `defect_detector` 输出缺陷结果

> 当前方向已改回 `fa97514` 之前的巡检/视觉检测方向。抓取方向相关的 `grasp_perception` / `gripper_driver` / 抓取 pipeline 不再是主线。当前论文和演示不依赖网页前端，也不依赖 fake driver。

## 常用命令

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install

colcon test
colcon test-result --verbose

source install/setup.bash
ros2 launch inspection_bringup drivers.launch.py
ros2 launch inspection_bringup system.launch.py
```

单包调试：

```bash
ros2 launch agv_driver agv_driver.launch.py
ros2 launch arm_driver arm_driver.launch.py
ros2 launch hikvision_driver hikvision_driver.launch.py
ros2 launch realsense_driver realsense.launch.py
ros2 launch arm_controller arm_controller.launch.py
ros2 launch task_coordinator task_coordinator.launch.py
```

## 代码规范

**私有成员变量必须以下划线 `_` 开头，禁止使用 `_` 结尾。**

```cpp
// 正确
int _count;
rclcpp::Publisher::SharedPtr _pub;

// 错误
int count_;
rclcpp::Publisher::SharedPtr pub_;
```

Node 只做 IO 与调度；协议细节、算法细节和业务分支应拆到 Core / Service / Adapter 类中，便于测试和论文复现实验。

## 主线包职责

```text
src/
├── agv_driver/          # AGV 底盘 TCP 驱动
├── arm_driver/          # 机械臂底层驱动
├── arm_controller/      # MoveIt2 运动控制
├── realsense_driver/    # RealSense 深度相机，工作距微调用
├── hikvision_driver/    # 海康工业相机，缺陷图像采集
├── defect_detector/     # 图像缺陷检测算法
├── task_coordinator/    # 巡检任务状态机
├── inspection_interface/# ROS2 msg/srv
├── inspection_bringup/  # launch 与配置
└── inspection_supervisor/# 可选健康监控
```

非主线保留包：

- `inspection_gateway`：历史 FastAPI 网关；当前无网页主线，不作为论文必需。
- `inspection_sim`：历史 fake driver 联调；当前不作为论文主线。
- `dio_driver`：DIO 通用驱动；巡检主线通常不需要。

## 巡检 pipeline

```text
IDLE
  -> MOVING_TO_STATION
  -> ARM_PRESET
  -> DEPTH_ADJUST
  -> CAPTURING
  -> 下一站位 / COMPLETED
```

每站配置来自 `inspection_bringup/config/inspection_stations.yaml`：

- `agv_pose`：AGV 导航站位
- `arm_joints`：机械臂预设观测关节角
- `target_distance`：目标工作距
- `distance_tolerance`：工作距容差
- `adjust_axis`：微调方向

## TODO 维护

- 仓库根 `TODO.md` 是“还要做什么”的单一事实来源。
- 完成任务必须勾选，并在提交信息中注明验证方式（真机/回放/单测）。
- 修改 public ROS API 或端到端数据流时，必须同步更新：
  - 相关包的 `src/<package>/AGENTS.md` / `CLAUDE.md` / `README.md`
  - `docs/WORKSPACE_OVERVIEW.md`
  - `docs/ARCHITECTURE.md`
  - `docs/IMPLEMENTATION_STATUS.md`
  - `TODO.md`

## Git Commit 规范

提交信息使用：

```text
<emoji> <type>[optional scope]: <中文描述>
```

常用类型：

- ✨ `feat`: 新增功能
- 🐛 `fix`: 修复 bug
- 📝 `docs`: 文档更新
- ♻️ `refactor`: 重构
- ✅ `test`: 测试
- 🔧 `chore`: 工具/依赖
- 🔨 `build`: 构建

禁止添加任何 AI 生成标记或 Co-authored-by 标记。

