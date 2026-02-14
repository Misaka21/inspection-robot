cmake_minimum_required(VERSION 3.8)
project(pose_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(inspection_interface REQUIRED)

add_executable(pose_detector_node src/pose_detector_node.cpp)
target_include_directories(pose_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_detector_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pose_detector_node
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "sensor_msgs"
  "image_transport"
  "tf2"
  "tf2_ros"
  "inspection_interface"
)

install(TARGETS pose_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
