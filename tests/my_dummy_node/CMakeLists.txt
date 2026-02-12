cmake_minimum_required(VERSION 3.8)
project(my_dummy_node)

# C++ 표준 설정 (C++17)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 컴파일러 경고 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(multi_agent_msgs REQUIRED) # 사용자 정의 메시지
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. Include 디렉토리 추가
include_directories(include)

# 3. 실행 파일 생성
# main.cpp와 dummy_agent_publisher.cpp를 함께 컴파일
add_executable(dummy_publisher_node
  src/main.cpp
  src/dummy_agent_publisher.cpp
)

# 4. 타겟 의존성 연결
ament_target_dependencies(dummy_publisher_node
  rclcpp
  multi_agent_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# 5. 설치 (Install) 규칙

# 실행 파일 설치
install(TARGETS dummy_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Launch 및 Config 파일 설치 (share 디렉토리로)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 6. 패키지 내보내기
ament_package()