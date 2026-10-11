cmake_minimum_required(VERSION 3.8)
project(replan_monitor)

# ---------- C++ 표준 ----------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---------- 경고 옵션 (선택) ----------
add_compile_options(-Wall -Wextra -Wpedantic)

# ---------- ROS2 의존성 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_common REQUIRED)
find_package(robot_interfaces REQUIRED)

# launch 패키지는 실행 파일이 아니라 launch 파일을
# share 디렉터리에 복사하기 위해 필요.
# find_package(launch REQUIRED)
# find_package(launch_ros REQUIRED)

# ---------- 헤더 디렉터리 ----------
include_directories(
  include
)

# -------------------------------------------------------------
#  1) 공통 라이브러리 (replan_monitor_node)
# -------------------------------------------------------------
add_library(replan_monitor_node SHARED
  src/replan_monitor_node.cpp
)

target_include_directories(replan_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(replan_monitor_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
)

# -------------------------------------------------------------
#  2) 실행 파일 : replan_monitor_main
#    - 위 라이브러리를 링크
# -------------------------------------------------------------
add_executable(replan_monitor_main src/main.cpp)

target_link_libraries(replan_monitor_main
  replan_monitor_node   # 라이브러리 연결
)

ament_target_dependencies(replan_monitor_main
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  nav2_msgs
)

# -------------------------------------------------------------
#  3) 실행 파일 : path_validator_node
#    - 자체 소스 (두 파일) 로 빌드
# -------------------------------------------------------------
add_executable(path_validator_node
  src/path_validator_node.cpp
  src/path_validator_main.cpp
)

ament_target_dependencies(path_validator_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_costmap_2d
  robot_interfaces
)

# -------------------------------------------------------------
#  설치 (install) 설정
# -------------------------------------------------------------
# 라이브러리와 실행 파일 모두 lib/에 배치
install(TARGETS
  replan_monitor_node
  replan_monitor_main
  path_validator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include
)

# launch, config 디렉터리 설치 (옵션)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ament 패키지 선언
ament_package()
