cmake_minimum_required(VERSION 3.16)
project(multi_agent_nav2)

# 반드시 ament_cmake 먼저
find_package(ament_cmake REQUIRED)

# 의존 패키지들
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(multi_agent_msgs REQUIRED)  # 여러분의 메시지 패키지
find_package(rclcpp_lifecycle REQUIRED)


# 라이브러리 타겟
add_library(${PROJECT_NAME} SHARED
  src/agent_layer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle nav2_costmap_2d pluginlib geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs multi_agent_msgs
)

# pluginlib에 XML 등록 (그룹: nav2_costmap_2d)
pluginlib_export_plugin_description_file(nav2_costmap_2d agent_layer_plugin.xml)

# 설치
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(FILES agent_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp rclcpp_lifecycle nav2_costmap_2d pluginlib geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs multi_agent_msgs
)

ament_package()
