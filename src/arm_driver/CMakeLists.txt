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
