cmake_minimum_required(VERSION 3.8)
project(sp_nav_bt)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_msg REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  robot_msg
  tf2_ros
  tf2
  tf2_geometry_msgs
)
include_directories(include)
add_library(nav_compute_path_service SHARED plugins/compute_path_service.cpp)
add_library(nav_state_track SHARED plugins/nav_state_track.cpp)
add_library(nav_rate_control SHARED plugins/rate_control.cpp)
add_library(nav_repeat_sequence SHARED plugins/repeat_sequence.cpp)
add_library(nav_get_pose SHARED plugins/get_pose.cpp)
add_library(nav_set_control_enable SHARED plugins/set_control_enable.cpp)
set(plugin_libs
  nav_compute_path_service
  nav_state_track
  nav_rate_control
  nav_repeat_sequence
  nav_get_pose
  nav_set_control_enable
)
foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()
add_library(nav_bt_engine_lib
  src/nav_bt_engine.cpp
)
add_executable(nav_interface_node
  src/nav_interface_node.cpp
)
add_executable(nav_action_client
  src/nav_action_client.cpp
)
target_link_libraries(nav_interface_node nav_bt_engine_lib)
target_link_libraries(nav_action_client nav_bt_engine_lib)
ament_target_dependencies(nav_bt_engine_lib ${dependencies})
ament_target_dependencies(nav_interface_node rclcpp rclcpp_action robot_msg)
ament_target_dependencies(nav_action_client rclcpp rclcpp_action robot_msg geometry_msgs std_msgs)
install(TARGETS
  ${plugin_libs}
  nav_bt_engine_lib
  nav_interface_node
  nav_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch nav_bt
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
