cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
# c++

add_executable(camera_draw_detections_node src/camera_draw_detections_node.cpp)
add_executable(camera_logger_node src/camera_logger_node.cpp)

ament_target_dependencies(camera_draw_detections_node
  rclcpp
  cv_bridge
  sensor_msgs
  OpenCV
  image_transport
  apriltag_msgs
)

ament_target_dependencies(camera_logger_node
  rclcpp
  cv_bridge
  sensor_msgs
  OpenCV
  image_transport
  drone_interfaces
)

install(TARGETS 
  camera_draw_detections_node
  camera_logger_node
  DESTINATION lib/${PROJECT_NAME})

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  camera/camera_calibration_node.py
  camera/camera_draw_node.py
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
