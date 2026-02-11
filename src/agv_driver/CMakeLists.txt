cmake_minimum_required(VERSION 3.8)
project(agv_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(inspection_interface REQUIRED)

add_executable(agv_driver_node
  src/agv_driver_node.cpp
  src/agv_client.cpp
  src/agv_transport.cpp
)
target_include_directories(agv_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(agv_driver_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  agv_driver_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "inspection_interface"
)

install(TARGETS agv_driver_node
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
