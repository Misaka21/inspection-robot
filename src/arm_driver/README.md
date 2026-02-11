# arm_driver

`arm_driver` 是 inspection-robot 的机械臂驱动包，职责是：

- 通过 EtherCAT 与 Elfin5 硬件通信
- 对外提供 ROS 话题/服务接口
- 不负责 MoveIt2 规划、IK 求解（这些属于 `arm_controller`）

## 目录结构

```text
arm_driver/
├── elfin_core/
│   ├── elfin_ethercat_driver/   # 底层 EtherCAT 驱动库
│   └── soem_ros2/               # SOEM 协议栈
├── include/
│   └── arm_driver/
│       └── arm_driver_node.hpp  # 驱动封装类声明
├── src/
│   ├── arm_driver_node.cpp      # main 入口
│   ├── arm_driver_node_core.cpp # EtherCAT 核心初始化与换算
│   └── arm_driver_node_ros.cpp  # ROS 话题/服务封装
├── config/
│   └── arm_driver.yaml          # 参数配置
└── launch/
    └── arm_driver.launch.py
```

说明：`elfin_core` 只放底层 C/C++ 驱动能力。ROS 接口实现在 `src/`，并通过 `include/arm_driver/arm_driver_node.hpp` 组织。

## 功能说明

- 接收关节目标并下发到 EtherCAT
- 发布关节状态（位置/速度/力矩）
- 发布机械臂状态（`inspection_interface/ArmStatus`）
- 提供使能、去使能、清故障、停止服务

## ROS 接口

节点名和命名空间：

- Executable: `arm_driver_node`
- Node Name: `arm_driver`（由 `launch/arm_driver.launch.py` 指定）
- Namespace: `/inspection/arm`

### 订阅

- `~/joint_cmd` (`sensor_msgs/msg/JointState`)
  - 支持两种输入：
    - 带 `name[]`：按关节名匹配
    - 不带 `name[]`：按 `command_joint_names` 顺序匹配

### 发布

- `/joint_states` (`sensor_msgs/msg/JointState`)
- `~/status` (`inspection_interface/msg/ArmStatus`)

### 服务

- `~/enable` (`std_srvs/srv/Trigger`)
- `~/disable` (`std_srvs/srv/Trigger`)
- `~/clear_fault` (`std_srvs/srv/Trigger`)
- `~/stop` (`std_srvs/srv/Trigger`)

## 参数说明（arm_driver.yaml）

核心参数：

- `elfin_ethernet_name`: EtherCAT 网卡名（默认 `eth0`）
- `state_publish_rate_hz`: `/joint_states` 发布频率
- `status_publish_rate_hz`: `~/status` 发布频率
- `motion_threshold`: 运动判定阈值
- `stop_disable_motors`: `~/stop` 后是否自动去使能
- `command_joint_names`: 无 `name[]` 指令时的输入关节顺序

底层 EtherCAT 参数（由 `elfin_ethercat_driver` 读取）：

- `slave_no`
- `joint_names`
- `reduction_ratios`
- `axis_position_factors`
- `axis_torque_factors`
- `count_zeros`

重要：

- 当前配置里 `count_zeros` 是示例值，必须替换为真机标定值。
- 厂商参数顺序是 `[J2, J1, J3, J4, J5, J6]`，代码里已做映射。

## soem_ros2 如何被调用

调用关系是进程内库调用，不是 ROS 话题调用：

1. `arm_driver_node.cpp` 进入 `ArmDriverNode`，在 `src/arm_driver_node_core.cpp` 内创建 `EtherCatManager`
2. `EtherCatManager` 在 `elfin_ethercat_manager.cpp` 中调用 SOEM C API（`ec_init/ec_config_init/ec_send_processdata/ec_receive_processdata`）
3. `ElfinEtherCATDriver/ElfinEtherCATClient` 基于 PDO/SDO 读写关节状态与目标

编译依赖链：

- `arm_driver` 依赖 `elfin_ethercat_driver`
- `elfin_ethercat_driver` 依赖 `soem_ros2`

## 启动方式

```bash
ros2 launch arm_driver arm_driver.launch.py
```

默认会加载：

- `config/arm_driver.yaml`

## 构建

在工作空间根目录（包含 `inspection-robot/src`）执行：

```bash
colcon build --packages-select soem_ros2 elfin_ethercat_driver arm_driver
```

## 与控制层的边界

- `arm_driver`: 只做硬件通信与状态上报
- `arm_controller`: 负责位姿目标、IK、MoveIt2 轨迹规划

推荐调用链：

`task_coordinator -> arm_controller -> arm_driver -> EtherCAT`

## 常见问题

1. 启动时报网卡错误：检查 `elfin_ethernet_name` 是否对应 EtherCAT 实际网口。
2. 机械臂不动且 `status.error_code=DISABLED`：先调用 `~/enable`。
3. 状态异常或报警：先调用 `~/clear_fault`，再 `~/enable`。
4. 关节跳变明显：优先检查 `count_zeros`、`joint_names` 顺序是否与真机一致。
