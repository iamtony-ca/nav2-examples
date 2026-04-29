cmake_minimum_required(VERSION 3.8)
project(nav2_stable_stopped_goal_checker)

# -----------------------
# Compile options
# -----------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Optional but recommended for plugins
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------
# Dependencies
# -----------------------
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)

# tf2_geometry_msgs는 현재 코드에서 직접 사용하지 않으므로 생략 가능.
# 필요하면 아래 2줄 주석 해제하고 ament_target_dependencies에도 추가해도 됨.
# find_package(tf2_geometry_msgs REQUIRED)

# -----------------------
# Library (plugin)
# -----------------------
add_library(${PROJECT_NAME} SHARED
  src/stable_stopped_goal_checker.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ament 방식으로 의존성 연결 (target_link_libraries 대신 권장)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  angles
  tf2
  # tf2_geometry_msgs
)

# -----------------------
# pluginlib export
# -----------------------
# plugins.xml을 ament index에 등록해 런타임 플러그인 로딩이 안정적
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# -----------------------
# Install
# -----------------------
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------
# Export for downstream packages
# -----------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  angles
  tf2
  # tf2_geometry_msgs
)

ament_package()