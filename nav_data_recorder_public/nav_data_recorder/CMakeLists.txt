cmake_minimum_required(VERSION 3.8)
project(nav_data_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_data_recorder_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(nav_data_codec SHARED
  src/scan_codec.cpp
  src/grid_codec.cpp)
target_include_directories(nav_data_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(nav_data_codec PUBLIC
  ${nav2_msgs_TARGETS} ${nav_data_recorder_msgs_TARGETS} ${sensor_msgs_TARGETS})

add_executable(nav_data_recorder_node
  src/recorder_node.cpp
  src/config.cpp
  src/event_log.cpp
  src/field_reader.cpp
  src/msg_definition.cpp)
target_include_directories(nav_data_recorder_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nav_data_recorder_node
  nav_data_codec
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rcpputils::rcpputils
  rmw::rmw
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
  ${action_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_data_recorder_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  tf2::tf2
  yaml-cpp)

install(TARGETS nav_data_codec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
add_executable(nav_data_fix_schema src/fix_schema.cpp src/msg_definition.cpp)
target_include_directories(nav_data_fix_schema PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nav_data_fix_schema
  ament_index_cpp::ament_index_cpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage)

install(TARGETS nav_data_recorder_node nav_data_fix_schema
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# tests need ament_cmake_gtest; robots without it still build (or use -DBUILD_TESTING=OFF)
find_package(ament_cmake_gtest QUIET)
if(BUILD_TESTING AND ament_cmake_gtest_FOUND)
  ament_add_gtest(test_codecs test/test_codecs.cpp)
  target_link_libraries(test_codecs nav_data_codec)
  ament_add_gtest(test_throttle test/test_throttle.cpp)
  target_include_directories(test_throttle PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_config test/test_config.cpp src/config.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../profiles)
  target_include_directories(test_config PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_config yaml-cpp)
  ament_add_gtest(test_msg_definition test/test_msg_definition.cpp src/msg_definition.cpp)
  target_include_directories(test_msg_definition PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_msg_definition ament_index_cpp::ament_index_cpp)
endif()

ament_package()
