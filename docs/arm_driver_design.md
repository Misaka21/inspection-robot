# arm_driver 包装层设计文档

## 1. 架构概览

```
arm_driver/
├── CMakeLists.txt
├── package.xml
├── elfin_core/                    # Elfin SDK 原始包（直接复制）
│   ├── soem_ros2/                 # EtherCAT 协议栈
│   ├── elfin_ethercat_driver/     # EtherCAT 硬件驱动
│   ├── elfin_ros_control/         # ROS2 control 硬件接口
│   └── elfin_robot_msgs/          # 消息定义
├── include/arm_driver/
│   └── arm_driver_wrapper.hpp     # 包装器头文件
└── src/
    ├── arm_driver_wrapper.cpp     # 包装器实现
    └── arm_driver_node.cpp        # ROS2 节点入口
```

## 2. 功能定义

### 2.1 arm_driver 职责

**仅负责硬件通信，不包含运动规划：**

- ✅ EtherCAT 协议通信
- ✅ 关节状态读取（位置、速度、扭矩）
- ✅ 关节命令发送（位置、速度）
- ✅ 使能/禁用机械臂
- ✅ 故障清除
- ✅ 紧急停止
- ❌ 运动规划（由 arm_controller 负责）
- ❌ 碰撞检测（由 arm_controller 负责）

### 2.2 接口设计

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/joint_cmd` | sensor_msgs/JointState | 关节位置/速度命令 |

#### 发布话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 250Hz | 关节状态（通过 elfin_ros_control 自动发布） |
| `~/status` | inspection_interface/ArmStatus | 10Hz | 机械臂状态（使能、故障、错误码） |

#### 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `~/enable` | std_srvs/Trigger | 使能机械臂 |
| `~/disable` | std_srvs/Trigger | 禁用机械臂 |
| `~/clear_fault` | std_srvs/Trigger | 清除故障 |
| `~/stop` | std_srvs/Empty | 紧急停止 |

## 3. 实现方案

### 3.1 包装器类设计

```cpp
class ArmDriverWrapper : public rclcpp::Node
{
public:
    ArmDriverWrapper(const rclcpp::NodeOptions& options);
    ~ArmDriverWrapper();

private:
    // ROS2 接口
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    rclcpp::Publisher<inspection_interface::msg::ArmStatus>::SharedPtr status_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

    rclcpp::TimerBase::SharedPtr status_timer_;

    // Elfin 驱动客户端（通过服务调用）
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr elfin_enable_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr elfin_clear_fault_client_;

    // 状态缓存
    bool is_enabled_;
    bool has_fault_;
    std::string error_message_;

    // 回调函数
    void jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void statusTimerCallback();

    void handleEnable(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleDisable(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleClearFault(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleStop(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
};
```

### 3.2 关键实现逻辑

#### 使能机械臂

```cpp
void ArmDriverWrapper::handleEnable(...)
{
    // 调用 elfin_ethercat_driver 的使能服务
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = elfin_enable_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        response->success = result->success;
        response->message = result->message;
        is_enabled_ = result->success;
    }
    else
    {
        response->success = false;
        response->message = "Failed to call elfin enable service";
    }
}
```

#### 状态发布

```cpp
void ArmDriverWrapper::statusTimerCallback()
{
    auto msg = inspection_interface::msg::ArmStatus();
    msg.header.stamp = this->now();
    msg.connected = true;  // 连接状态（从 elfin_ethercat_driver 获取）
    msg.enabled = is_enabled_;
    msg.has_fault = has_fault_;
    msg.error_message = error_message_;

    status_pub_->publish(msg);
}
```

## 4. CMakeLists.txt 配置

```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_driver)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(inspection_interface REQUIRED)

# 包含 elfin_core
add_subdirectory(elfin_core/soem_ros2)
add_subdirectory(elfin_core/elfin_robot_msgs)
add_subdirectory(elfin_core/elfin_ethercat_driver)
add_subdirectory(elfin_core/elfin_ros_control)

# 包装器库
add_library(${PROJECT_NAME}_wrapper SHARED
  src/arm_driver_wrapper.cpp
)

target_include_directories(${PROJECT_NAME}_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_wrapper
  rclcpp
  sensor_msgs
  std_srvs
  inspection_interface
)

# 可执行文件
add_executable(arm_driver_node src/arm_driver_node.cpp)
target_link_libraries(arm_driver_node ${PROJECT_NAME}_wrapper)

# 安装
install(TARGETS
  arm_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
```

## 5. package.xml 配置

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm_driver</name>
  <version>0.1.0</version>
  <description>Elfin arm EtherCAT driver wrapper</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <depend>inspection_interface</depend>

  <!-- Elfin core 依赖 -->
  <depend>soem_ros2</depend>
  <depend>elfin_robot_msgs</depend>
  <depend>elfin_ethercat_driver</depend>
  <depend>elfin_ros_control</depend>
  <depend>controller_manager</depend>
  <depend>hardware_interface</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 6. 启动流程

### 6.1 启动顺序

```bash
# 1. 启动底层 EtherCAT 驱动和 ROS2 control
ros2 launch arm_driver elfin_bringup.launch.py

# 2. 启动包装器节点
ros2 run arm_driver arm_driver_node
```

### 6.2 Launch 文件示例

```python
# arm_driver/launch/arm_driver.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 加载 elfin_ros_control (硬件接口)
    elfin_controller = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=['config/elfin_arm_control.yaml'],
        output='screen'
    )

    # 加载 joint_trajectory_controller
    load_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['elfin_arm_controller', 'joint_state_broadcaster'],
        output='screen'
    )

    # 包装器节点
    wrapper_node = Node(
        package='arm_driver',
        executable='arm_driver_node',
        namespace='/inspection/arm',
        output='screen'
    )

    return LaunchDescription([
        elfin_controller,
        load_controller,
        wrapper_node
    ])
```

## 7. 测试验证

### 7.1 使能测试

```bash
# 调用使能服务
ros2 service call /inspection/arm/enable std_srvs/srv/Trigger

# 查看状态
ros2 topic echo /inspection/arm/status
```

### 7.2 关节命令测试

```bash
# 发送关节命令
ros2 topic pub /inspection/arm/joint_cmd sensor_msgs/msg/JointState \
  "{name: ['elfin_joint1', 'elfin_joint2', 'elfin_joint3', 'elfin_joint4', 'elfin_joint5', 'elfin_joint6'],
    position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 7.3 状态监控

```bash
# 查看关节状态
ros2 topic echo /joint_states

# 查看机械臂状态
ros2 topic echo /inspection/arm/status
```

## 8. 注意事项

1. **EtherCAT 权限**：需要 root 权限或配置 udev 规则
2. **实时性**：确保系统支持 250Hz 更新率
3. **网络配置**：检查以太网口配置（在 `elfin_drivers.yaml` 中）
4. **从站号**：确认 EtherCAT 从站号正确（slave_no: [1, 2, 3]）
5. **零点标定**：首次使用需要标定零点（count_zeros 参数）
