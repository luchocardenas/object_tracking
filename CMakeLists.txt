cmake_minimum_required(VERSION 3.8)
project(object_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

add_library(config_loader_lib
  src/config_loader.cpp
)

target_include_directories(config_loader_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(config_loader_lib rclcpp yaml-cpp)

add_executable(sensor_data_publisher src/sensor_data_publisher_node.cpp)
ament_target_dependencies(sensor_data_publisher rclcpp ament_index_cpp geometry_msgs tf2 tf2_ros)

add_executable(transform_detections src/transform_detections_node.cpp)
ament_target_dependencies(transform_detections rclcpp ament_index_cpp geometry_msgs tf2 tf2_geometry_msgs)

add_executable(test_tf2 src/test_tf2.cpp)
ament_target_dependencies(test_tf2 rclcpp tf2)

# Link yaml-cpp explicitly
target_link_libraries(config_loader_lib yaml-cpp)
target_link_libraries(transform_detections yaml-cpp config_loader_lib)

# Install the data directory to the package's install space
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  FILES_MATCHING PATTERN "*.json"
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)

# Install the config directory to the package's install space
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)

install(TARGETS 
  config_loader_lib
  sensor_data_publisher 
  transform_detections
  test_tf2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
