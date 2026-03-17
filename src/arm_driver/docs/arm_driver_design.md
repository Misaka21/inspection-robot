# arm_driver 包装层设计文档

## 1. 架构概览

```
arm_driver/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── arm_driver.launch.py       # 启动入口
├── config/
│   └── arm_driver.yaml            # 参数配置
├── include/arm_driver/
│   ├── arm_driver_node.hpp
│   └── arm_driver_node_core.hpp
└── src/
    ├── arm_driver_node.cpp        # ROS2 节点入口
    ├── arm_driver_node_core.cpp   # 核心驱动逻辑
    └── arm_driver_node_ros.cpp    # ROS2 接口封装
```

依赖位于工作区 `src/elfin_sdk/` 下的独立包（例如 `elfin_ethercat_driver`、`soem_ros2`），`arm_driver` 通过 `find_package` 使用，不再内嵌 `elfin_core` 目录。

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
| `/joint_states` | sensor_msgs/JointState | 250Hz | 关节状态（由底层驱动链路发布） |
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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(inspection_interface REQUIRED)
find_package(elfin_ethercat_driver REQUIRED)

add_executable(arm_driver_node
  src/arm_driver_node.cpp
  src/arm_driver_node_core.cpp
  src/arm_driver_node_ros.cpp
)
target_include_directories(arm_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_driver_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  arm_driver_node
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "inspection_interface"
  "elfin_ethercat_driver"
)

install(TARGETS arm_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## 5. package.xml 配置

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm_driver</name>
  <version>1.0.0</version>
  <description>Robotic arm driver for the inspection system</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <depend>inspection_interface</depend>
  <depend>elfin_ethercat_driver</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 6. 启动流程

### 6.1 启动顺序

```bash
# 启动 arm_driver（默认命名空间 /inspection/arm）
ros2 launch arm_driver arm_driver.launch.py
```

### 6.2 Launch 文件示例

```python
# arm_driver/launch/arm_driver.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection/arm",
        description="Namespace for arm_driver node",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("arm_driver"),
            "config",
            "arm_driver.yaml",
        ),
        description="Path to ROS2 parameters file",
    )

    arm_driver_node = Node(
        package="arm_driver",
        executable="arm_driver_node",
        name="arm_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        arm_driver_node
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
