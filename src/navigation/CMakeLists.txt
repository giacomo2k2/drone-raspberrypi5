cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Dependencies =====
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)


# ===== C++ Node =====
add_executable(navigator_node src/navigator_node.cpp)
ament_target_dependencies(navigator_node
  rclcpp
  rclpy
  rclcpp_action
  mavros_msgs
  geometry_msgs
  drone_interfaces
  std_msgs
)
install(TARGETS navigator_node DESTINATION lib/${PROJECT_NAME})

# ===== Python Node =====

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS navigation/global_trajectory_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===== Tests =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
