cmake_minimum_required(VERSION 3.8)
project(amr_bt_nodes)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)  # For SmartRecoveryAction
find_package(nav_msgs REQUIRED)

find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)

find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(include)

# ───── Build run_if_flag_true_decorator ─────
add_library(run_if_flag_true_decorator SHARED
  src/run_if_flag_true_decorator.cpp)

ament_target_dependencies(run_if_flag_true_decorator
  rclcpp
  std_msgs
  behaviortree_cpp
  pluginlib)

target_include_directories(run_if_flag_true_decorator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ───── Build check_flag_condition ─────
add_library(check_flag_condition SHARED
  src/check_flag_condition.cpp)

ament_target_dependencies(check_flag_condition
  rclcpp
  std_msgs
  geometry_msgs
  behaviortree_cpp
  pluginlib)

target_include_directories(check_flag_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# ───── Build trigger_planner_decorator ─────
add_library(trigger_planner_decorator SHARED
  src/trigger_planner_decorator.cpp)

ament_target_dependencies(trigger_planner_decorator
  rclcpp
  std_msgs
  geometry_msgs
  behaviortree_cpp
  pluginlib)

target_include_directories(trigger_planner_decorator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build smart_recovery_action_node ─────
add_library(smart_recovery_action_node SHARED
  src/smart_recovery_action.cpp)

ament_target_dependencies(smart_recovery_action_node
  rclcpp rclcpp_action behaviortree_cpp nav2_behavior_tree robot_interfaces geometry_msgs pluginlib)

target_include_directories(smart_recovery_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build set_truncated_goal_from_path ─────
add_library(set_truncated_goal_from_path SHARED
  src/set_truncated_goal_from_path.cpp)

ament_target_dependencies(set_truncated_goal_from_path
  rclcpp rclcpp_action behaviortree_cpp nav2_behavior_tree robot_interfaces geometry_msgs pluginlib)

target_include_directories(set_truncated_goal_from_path PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build is_path_valid_condition ─────
add_library(is_path_valid_condition SHARED
  src/is_path_valid_condition.cpp)

ament_target_dependencies(is_path_valid_condition
  rclcpp behaviortree_cpp nav_msgs pluginlib)

target_include_directories(is_path_valid_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build prune_passed_path_action ─────
add_library(prune_passed_path_action SHARED
  src/prune_passed_path_action.cpp)

ament_target_dependencies(prune_passed_path_action
  rclcpp behaviortree_cpp nav2_common tf2_ros nav2_util pluginlib)

target_include_directories(prune_passed_path_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build prune_passed_path_action ─────
add_library(log_text_action SHARED
  src/log_text_action.cpp)

ament_target_dependencies(log_text_action
  rclcpp behaviortree_cpp pluginlib)

target_include_directories(log_text_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Pluginlib Export ─────
pluginlib_export_plugin_description_file(behaviortree_cpp plugin.xml)

# ───── Install Targets ─────
install(TARGETS
  run_if_flag_true_decorator
  check_flag_condition
  trigger_planner_decorator
  smart_recovery_action_node
  set_truncated_goal_from_path
  is_path_valid_condition
  prune_passed_path_action
  log_text_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ───── Install Headers ─────
install(DIRECTORY include/
  DESTINATION include)

# ───── Install Plugin Description ─────
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
