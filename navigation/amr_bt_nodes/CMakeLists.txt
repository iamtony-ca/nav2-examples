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
find_package(nav2_costmap_2d REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)

# find_package(robot_interfaces REQUIRED)

include_directories(
  include
  # ${rclcpp_INCLUDE_DIRS}
  # ${behaviortree_cpp_INCLUDE_DIRS}
  # ${nav2_behavior_tree_INCLUDE_DIRS}
  # ${nav2_util_INCLUDE_DIRS}
  # ${robot_interfaces_INCLUDE_DIRS} # <-- 이 부분이 중요!
  # ${nav_msgs_INCLUDE_DIRS}
  # ${geometry_msgs_INCLUDE_DIRS}
  # ${rclcpp_action_INCLUDE_DIRS}
  # ${tf2_ros_INCLUDE_DIRS}
  # ${nav2_common_INCLUDE_DIRS}
  
  )

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

# ───── Build is_path_valid_custom_condition ─────
add_library(is_path_valid_custom_condition SHARED
  src/is_path_valid_custom_condition.cpp)

ament_target_dependencies(is_path_valid_custom_condition
  rclcpp behaviortree_cpp nav_msgs pluginlib)

target_include_directories(is_path_valid_custom_condition PUBLIC
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

# ───── Build log_text_action ─────
add_library(log_text_action SHARED
  src/log_text_action.cpp)

ament_target_dependencies(log_text_action
  rclcpp behaviortree_cpp pluginlib)

target_include_directories(log_text_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build path_publisher_action ─────
add_library(path_publisher_action SHARED
  src/path_publisher_action.cpp)

ament_target_dependencies(path_publisher_action
  rclcpp behaviortree_cpp pluginlib nav_msgs)

target_include_directories(path_publisher_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build trigger_receiver_and_parser ─────
add_library(trigger_receiver_and_parser SHARED
  src/trigger_receiver_and_parser.cpp)

ament_target_dependencies(trigger_receiver_and_parser
  rclcpp behaviortree_cpp pluginlib std_msgs nav2_behavior_tree nav2_util robot_interfaces)

target_include_directories(trigger_receiver_and_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build is_blackboard_changed_condition ─────
add_library(is_blackboard_changed_condition SHARED
  src/is_blackboard_changed_condition.cpp)

ament_target_dependencies(is_blackboard_changed_condition
  rclcpp behaviortree_cpp pluginlib)

target_include_directories(is_blackboard_changed_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build string_publisher_action ─────
add_library(string_publisher_action SHARED
  src/string_publisher_action.cpp)

ament_target_dependencies(string_publisher_action
  rclcpp std_msgs behaviortree_cpp pluginlib)

target_include_directories(string_publisher_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build trigger_planner_goals_decorator ─────
add_library(trigger_planner_goals_decorator SHARED
  src/trigger_planner_goals_decorator.cpp)

ament_target_dependencies(trigger_planner_goals_decorator
  rclcpp
  std_msgs
  geometry_msgs
  behaviortree_cpp
  pluginlib)

target_include_directories(trigger_planner_goals_decorator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build get_next_few_goals_action ─────
add_library(get_next_few_goals_action SHARED
  src/get_next_few_goals_action.cpp)

ament_target_dependencies(get_next_few_goals_action
  rclcpp geometry_msgs behaviortree_cpp pluginlib)

target_include_directories(get_next_few_goals_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build remove_first_goal_action ─────
add_library(remove_first_goal_action SHARED
  src/remove_first_goal_action.cpp)

ament_target_dependencies(remove_first_goal_action
  rclcpp geometry_msgs behaviortree_cpp pluginlib)

target_include_directories(remove_first_goal_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build replan_guard_decorator ─────
add_library(replan_guard_decorator SHARED
  src/replan_guard_decorator.cpp)

ament_target_dependencies(replan_guard_decorator
  rclcpp std_msgs geometry_msgs behaviortree_cpp pluginlib)

target_include_directories(replan_guard_decorator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build advanced_remove_passed_goals_action ─────
add_library(advanced_remove_passed_goals_action SHARED
  src/advanced_remove_passed_goals_action.cpp)

ament_target_dependencies(advanced_remove_passed_goals_action
  rclcpp nav_msgs geometry_msgs nav2_util tf2_geometry_msgs behaviortree_cpp pluginlib)

target_include_directories(advanced_remove_passed_goals_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build is_goals_occupied_condition ─────
add_library(is_goals_occupied_condition SHARED
  src/is_goals_occupied_condition.cpp)

ament_target_dependencies(is_goals_occupied_condition
  rclcpp geometry_msgs tf2_ros nav2_msgs nav2_costmap_2d nav2_util tf2_geometry_msgs behaviortree_cpp pluginlib)

target_include_directories(is_goals_occupied_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build string_periodic_publisher_action ─────
add_library(string_periodic_publisher_action SHARED
  src/string_periodic_publisher_action.cpp)

ament_target_dependencies(string_periodic_publisher_action
  rclcpp std_msgs behaviortree_cpp pluginlib)

target_include_directories(string_periodic_publisher_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  



# ───── Build error_code_publisher_action ─────
add_library(error_code_publisher_action SHARED
  src/error_code_publisher_action.cpp)

ament_target_dependencies(error_code_publisher_action
  rclcpp std_msgs behaviortree_cpp pluginlib)

target_include_directories(error_code_publisher_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build run_n_times_decorator ─────
add_library(run_n_times_decorator SHARED
  src/run_n_times_decorator.cpp)

ament_target_dependencies(run_n_times_decorator
  rclcpp behaviortree_cpp pluginlib)

target_include_directories(run_n_times_decorator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build bool_publisher_action ─────
add_library(bool_publisher_action SHARED
  src/bool_publisher_action.cpp)

ament_target_dependencies(bool_publisher_action
  rclcpp std_msgs behaviortree_cpp pluginlib)

target_include_directories(bool_publisher_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build set_blackboard_bool_action ─────
add_library(set_blackboard_bool_action SHARED
  src/set_blackboard_bool_action.cpp)

ament_target_dependencies(set_blackboard_bool_action
  rclcpp std_msgs behaviortree_cpp pluginlib)

target_include_directories(set_blackboard_bool_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build set_blackboard_ushort_action ─────
add_library(set_blackboard_ushort_action SHARED
  src/set_blackboard_ushort_action.cpp)

ament_target_dependencies(set_blackboard_ushort_action
  rclcpp behaviortree_cpp pluginlib)

target_include_directories(set_blackboard_ushort_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build check_pause_condition ─────
add_library(check_pause_condition SHARED
  src/check_pause_condition.cpp)

ament_target_dependencies(check_pause_condition
  rclcpp
  std_msgs
  behaviortree_cpp
  pluginlib)

target_include_directories(check_pause_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# ───── Build reactive_round_robin_control ─────
add_library(reactive_round_robin_control SHARED
  src/reactive_round_robin_control.cpp)

ament_target_dependencies(reactive_round_robin_control
  rclcpp
  behaviortree_cpp
  pluginlib)

target_include_directories(reactive_round_robin_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# ───── Build check_pause_reset_condition ─────
add_library(check_pause_reset_condition SHARED
  src/check_pause_reset_condition.cpp)

ament_target_dependencies(check_pause_reset_condition
  rclcpp
  std_msgs
  behaviortree_cpp
  pluginlib)

target_include_directories(check_pause_reset_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# ───── Build check_final_goal_yaw_action ─────
add_library(check_final_goal_yaw_action SHARED
  src/check_final_goal_yaw_action.cpp)

ament_target_dependencies(check_final_goal_yaw_action
  rclcpp
  std_msgs
  behaviortree_cpp
  geometry_msgs
  tf2_geometry_msgs
  # tf2
  pluginlib)

target_include_directories(check_final_goal_yaw_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# ───── Build is_goals_occupied_agent_condition ─────
add_library(is_goals_occupied_agent_condition SHARED
  src/is_goals_occupied_agent_condition.cpp)

ament_target_dependencies(is_goals_occupied_agent_condition
  rclcpp geometry_msgs tf2_ros nav2_msgs nav2_costmap_2d nav2_util tf2_geometry_msgs behaviortree_cpp pluginlib)

target_include_directories(is_goals_occupied_agent_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

# ───── Build check_and_extract_spin_action ─────
add_library(check_and_extract_spin_action SHARED
  src/check_and_extract_spin_action.cpp)

ament_target_dependencies(check_and_extract_spin_action
  rclcpp geometry_msgs tf2 tf2_geometry_msgs behaviortree_cpp pluginlib)

target_include_directories(check_and_extract_spin_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

  
# ───── Build remove_passed_goal_action ─────
add_library(remove_passed_goal_action SHARED
  src/remove_passed_goal_action.cpp)

ament_target_dependencies(remove_passed_goal_action
  rclcpp behaviortree_cpp pluginlib geometry_msgs nav2_util)

target_include_directories(remove_passed_goal_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  


# ───── Build publish_remaining_goals_action ─────
add_library(publish_remaining_goals_action SHARED
  src/publish_remaining_goals_action.cpp)

ament_target_dependencies(publish_remaining_goals_action
  rclcpp behaviortree_cpp pluginlib geometry_msgs nav_msgs)

target_include_directories(publish_remaining_goals_action PUBLIC
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
  is_path_valid_custom_condition
  prune_passed_path_action
  log_text_action
  path_publisher_action
  trigger_receiver_and_parser
  is_blackboard_changed_condition
  string_publisher_action
  trigger_planner_goals_decorator
  get_next_few_goals_action
  remove_first_goal_action
  replan_guard_decorator
  advanced_remove_passed_goals_action
  is_goals_occupied_condition
  string_periodic_publisher_action
  error_code_publisher_action
  run_n_times_decorator
  bool_publisher_action
  set_blackboard_bool_action
  set_blackboard_ushort_action
  reactive_round_robin_control
  check_pause_reset_condition
  check_final_goal_yaw_action
  is_goals_occupied_agent_condition
  check_and_extract_spin_action
  remove_passed_goal_action
  publish_remaining_goals_action

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
