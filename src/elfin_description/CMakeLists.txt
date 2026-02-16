cmake_minimum_required(VERSION 3.8)
project(elfin_description)

# Disable symlink install (required for exfat filesystem)
set(AMENT_CMAKE_SYMLINK_INSTALL OFF)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
