cmake_minimum_required(VERSION 3.8)
project(robot_monitoring)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(status_manager_node
  src/main.cpp
  src/status_manager.cpp
)

# 컴파일 시 'include' 디렉토리를 헤더 검색 경로에 추가
target_include_directories(status_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(status_manager_node
  "rclcpp"
  "rclcpp_action"
  "nav2_msgs"
  "std_msgs"
)

# 'include' 디렉토리의 모든 파일을 설치(install)
install(DIRECTORY
  include/
  DESTINATION include
)

install(TARGETS
  status_manager_node
  DESTINATION lib/${PROJECT_NAME}
)


########## Launch directory #######################
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# ament_cmake에 패키지 정보를 알림
ament_package()