cmake_minimum_required(VERSION 3.5)
project(status_monitoring)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)

# Include directories
include_directories(
  include
)

# List of monitor source files
set(MONITOR_SRCS
  src/MAG_monitor.cpp
  src/GPS_monitor.cpp
  src/IMU_monitor.cpp
  src/estimator_monitor.cpp
  src/PWR_monitor.cpp
)

# Base monitor source
set(BASE_SRCS src/base_monitor.cpp)

# Main node source
set(NODE_SRCS src/status_monitoring_node.cpp)

# Create library for monitors
add_library(monitors_lib
  ${BASE_SRCS}
  ${MONITOR_SRCS}
)
ament_target_dependencies(monitors_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  drone_interfaces
)

# Create executable for node
add_executable(status_monitoring_node_exec
  ${NODE_SRCS}
)
ament_target_dependencies(status_monitoring_node_exec
  rclcpp
  sensor_msgs
  geometry_msgs
  drone_interfaces
)
target_link_libraries(status_monitoring_node_exec monitors_lib)

# Install
install(TARGETS
  status_monitoring_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export package
ament_package()
