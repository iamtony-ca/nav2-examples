cmake_minimum_required(VERSION 3.8)
project(yield_bt_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)

# BT 플러그인 라이브러리 1개 = 노드 1개 (amr_bt_nodes 와 같은 방식, plugin_lib_names 에 이름을 적는다)
add_library(get_yield_goal_action SHARED src/get_yield_goal_action.cpp)
ament_target_dependencies(get_yield_goal_action
  rclcpp geometry_msgs std_msgs behaviortree_cpp nav2_behavior_tree pluginlib)
target_compile_definitions(get_yield_goal_action PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS get_yield_goal_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# [V2.36b 09-26] 단위시험: 조정 계층과 같은 QoS 로 쏠 때 YieldFlagFresh / GetYieldGoalAction 동작
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yield_bt_nodes test/test_yield_bt_nodes.cpp TIMEOUT 60)
  ament_target_dependencies(test_yield_bt_nodes rclcpp geometry_msgs std_msgs behaviortree_cpp)
  target_compile_definitions(test_yield_bt_nodes PRIVATE
    PLUGIN_PATH="$<TARGET_FILE:get_yield_goal_action>")
  add_dependencies(test_yield_bt_nodes get_yield_goal_action)
endif()

ament_package()
