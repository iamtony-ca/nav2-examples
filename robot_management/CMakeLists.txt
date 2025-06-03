cmake_minimum_required(VERSION 3.10)
project(robot_management)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# ▶ 1. 라이브러리 생성: task_management_lib
add_library(task_management_lib
  src/task_management.cpp
  src/nav2_param_manager.cpp
)

ament_target_dependencies(task_management_lib
  rclcpp
  rclcpp_action
  rcl_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  std_msgs
  nav2_msgs
  geometry_msgs
)

target_include_directories(task_management_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ▶ 2. 실행 파일 생성: task_management_node (main.cpp 필요)
add_executable(task_management_node src/main.cpp)
target_link_libraries(task_management_node task_management_lib)

ament_target_dependencies(task_management_node
  rclcpp
)

# ▶ 설치
install(TARGETS
  task_management_node
  task_management_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# (선택) launch 폴더
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
