cmake_minimum_required(VERSION 3.8)
project(task_coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(inspection_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(task_coordinator_node src/task_coordinator_node.cpp)
target_include_directories(task_coordinator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  task_coordinator_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "std_srvs"
  "inspection_interface"
  "tf2"
)
target_link_libraries(task_coordinator_node yaml-cpp)

install(TARGETS task_coordinator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/demo_inspection.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_inspection)

install(PROGRAMS scripts/jog_arm.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jog_arm)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
