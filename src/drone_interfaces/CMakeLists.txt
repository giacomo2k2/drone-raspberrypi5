cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  # messages
  "msg/FailsafeEvent.msg"
  "msg/PreArmCheck.msg"
  "msg/SingleComponentStatus.msg"
  "msg/ComponentsStatus.msg"
  "msg/ApriltagPosition.msg"
  "msg/CompanionComputerInfo.msg"
  "msg/CompanionComputerSM.msg"
  "msg/Mission.msg"

  # services
  "srv/GenerateGlobalTrajectory.srv"
  "srv/Arm.srv"
  "srv/Disarm.srv"
  "srv/Land.srv"
  "srv/Takeoff.srv"
  "srv/ActivateDetector.srv"
  "srv/StartMission.srv"
  "srv/AbortMission.srv"
  "srv/Shutdown.srv"
  "srv/StopRecording.srv"

  # actions
  "action/FollowTrajectory.action"

  # missions
  "msg/MissionData.msg"
  "msg/SetpointNavigation.msg"

  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces diagnostic_msgs geographic_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()