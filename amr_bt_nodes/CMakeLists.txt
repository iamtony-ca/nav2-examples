cmake_minimum_required(VERSION 3.8)
project(amr_bt_nodes)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

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

pluginlib_export_plugin_description_file(behaviortree_cpp plugin.xml)

install(TARGETS run_if_flag_true_decorator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()