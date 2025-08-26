cmake_minimum_required(VERSION 3.8)
project(velocity_modifier)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 종속성 패키지 검색
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)  # robot_interfaces 패키지 추가
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 커스텀 메시지(.msg)로부터 C++ 코드를 생성
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/ModifierControl.msg"
# )

# 헤더 파일이 있는 include 디렉토리 추가
include_directories(include)

# 실행 파일 생성 (main.cpp와 node.cpp를 함께 컴파일)
add_executable(
  velocity_modifier_node
  src/main.cpp
  src/velocity_modifier_node.cpp
)

# 생성된 메시지 코드에 대한 링크를 포함한 의존성 설정
ament_target_dependencies(
  velocity_modifier_node
  rclcpp
  geometry_msgs
  std_msgs
  robot_interfaces
  rosidl_default_runtime
)

# 노드가 메시지 생성보다 나중에 빌드되도록 의존성 관계 설정
# add_dependencies(velocity_modifier_node ${PROJECT_NAME}_EXPORTED_TARGETS)

# 설치 규칙
install(
  TARGETS velocity_modifier_node
  DESTINATION lib/${PROJECT_NAME}
)

# msg 디렉토리와 생성된 메시지 헤더 설치
# install(
#   # DIRECTORY msg
#   DESTINATION share/${PROJECT_NAME}
# )
install(
  DIRECTORY include/
  DESTINATION include
)

########## Launch directory ##################################################
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# ament_cmake에 패키지 정보를 알림
ament_package()