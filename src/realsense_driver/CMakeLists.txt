cmake_minimum_required(VERSION 3.8)
project(realsense_driver)

# This is a meta-package that only provides launch files and config
# The actual driver comes from ros-humble-realsense2-camera system package

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}/
)
