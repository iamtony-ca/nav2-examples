cmake_minimum_required(VERSION 3.8)
project(replan_monitor)

# ───── Use C++17 standard ─────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ───── Dependencies ─────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
# find_package(launch REQUIRED)
# find_package(launch_ros REQUIRED)


include_directories(include)

# ───── Build library (replan_monitor_node) ─────
add_library(replan_monitor_node SHARED
  src/replan_monitor_node.cpp
)

ament_target_dependencies(replan_monitor_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
)

target_include_directories(replan_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ───── Build main executable using the library ─────
add_executable(replan_monitor_main src/main.cpp)

# FIXED: link to actual library target name, not project name
target_link_libraries(replan_monitor_main
  replan_monitor_node
)

ament_target_dependencies(replan_monitor_main
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
)


# Executable for path_validator_node
add_executable(path_validator_node
  src/path_validator_node.cpp
  src/path_validator_main.cpp
)

ament_target_dependencies(path_validator_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_costmap_2d
)

# install(TARGETS path_validator_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )







# ───── Install targets ─────
install(TARGETS
  replan_monitor_node
  replan_monitor_main
  path_validator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # <-- .so 실행 파일
)

# ───── Install headers ─────
install(DIRECTORY include/
  DESTINATION include)

# ───── Install launch files (optional) ─────
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
