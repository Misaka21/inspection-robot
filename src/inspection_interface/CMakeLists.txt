cmake_minimum_required(VERSION 3.8)
project(inspection_interface)

# Disable symlink install (required for exfat filesystem)
set(AMENT_CMAKE_SYMLINK_INSTALL OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/InspectionTask.msg"
  "msg/InspectionResult.msg"
  "msg/DefectInfo.msg"
  "msg/RobotPose.msg"
  "msg/AgvStatus.msg"
  "msg/ArmStatus.msg"
  "msg/SystemState.msg"
  # Services
  "srv/StartInspection.srv"
  "srv/StopInspection.srv"
  "srv/PauseInspection.srv"
  "srv/ResumeInspection.srv"
  "srv/GetInspectionStatus.srv"
  "srv/MoveToPose.srv"
  "srv/GetNavMap.srv"
  "srv/PlanInspection.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
