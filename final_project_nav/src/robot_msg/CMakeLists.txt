cmake_minimum_required(VERSION 3.8)
project(robot_msg)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotKinematics.msg"
  "msg/RobotKinematicsArray.msg"
  "msg/GimbalControlMsg.msg"
  "msg/ChassisModeMsg.msg"
  "action/NavigateToPose.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)
ament_package()
