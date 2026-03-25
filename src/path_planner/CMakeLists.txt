cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(inspection_interface REQUIRED)

# 创建库
add_library(${PROJECT_NAME}_core STATIC
  src/stance_sampler.cpp
  src/ik_adapter.cpp
  src/planner_core.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
ament_target_dependencies(
  ${PROJECT_NAME}_core
  "rclcpp"
  "geometry_msgs"
  "moveit_ros_planning_interface"
  "moveit_core"
  "tf2_eigen"
)

# 创建可执行节点
add_executable(path_planner_node src/path_planner_node.cpp)
target_include_directories(path_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_planner_node PUBLIC cxx_std_17)
target_link_libraries(path_planner_node ${PROJECT_NAME}_core)
ament_target_dependencies(
  path_planner_node
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "moveit_ros_planning_interface"
  "moveit_core"
  "tf2_eigen"
  "inspection_interface"
)

install(TARGETS path_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # StanceSampler 单元测试（纯几何，无ROS依赖）
  ament_add_gtest(test_stance_sampler test/test_stance_sampler.cpp)
  target_include_directories(test_stance_sampler PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_stance_sampler ${PROJECT_NAME}_core)
  ament_target_dependencies(test_stance_sampler
    "geometry_msgs"
  )
endif()

ament_package()
