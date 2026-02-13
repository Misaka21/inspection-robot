cmake_minimum_required(VERSION 3.5)
project(elfin_ethercat_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(INSTALL_LIB_DIR "${PROJECT_BINARY_DIR}/lib")

set(CMAKE_SKIP_BUILD_RPATH FALSE)

set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${PROJECT_BINARY_DIR}")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(soem_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  /usr/include
)

add_library(${PROJECT_NAME}_lib
  SHARED
  "src/elfin_ethercat_driver.cpp"
  "src/elfin_ethercat_client.cpp"
  "src/elfin_ethercat_manager.cpp"
  include/${PROJECT_NAME}/elfin_ethercat_driver.h
  include/${PROJECT_NAME}/elfin_ethercat_client.h
  include/${PROJECT_NAME}/elfin_ethercat_manager.h
)

target_link_libraries(${PROJECT_NAME}_lib Boost::system Boost::thread)

set(DEPENDENCIES
  rclcpp
  std_msgs
  soem_ros2
  std_srvs
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
    DIRECTORY 
    src
    DESTINATION 
    share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
