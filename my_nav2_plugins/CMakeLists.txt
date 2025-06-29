cmake_minimum_required(VERSION 3.8)
project(my_nav2_plugins)

# C++ 표준 및 컴파일러 경고 설정 (표준적인 모범 사례)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 빌드에 필요한 ROS 2 패키지를 찾습니다.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)

# 두 개의 C++ 소스 파일을 포함하는 하나의 공유 라이브러리를 생성합니다.
# 이렇게 하면 관리가 용이합니다.
add_library(my_nav2_bt_nodes SHARED
  # src/check_battery_condition.cpp
  src/prune_passed_path_node.cpp
)

# ========================= 에러 해결을 위한 추가된 부분 =========================
# 이 라이브러리를 빌드할 때 'include' 디렉토리를 포함하도록 지정합니다.
# 이렇게 해야 .cpp 파일들이 자신의 .hpp 파일을 찾을 수 있습니다.
target_include_directories(my_nav2_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ============================================================================


# 생성된 라이브러리가 어떤 다른 라이브러리에 의존하는지 명시합니다.
# PUBLIC 키워드를 사용하면 include 경로가 올바르게 전파됩니다.
ament_target_dependencies(my_nav2_bt_nodes PUBLIC
  "rclcpp"
  "behaviortree_cpp"
  "pluginlib"
  "nav2_common"
  "nav2_util"
  "tf2_ros"
)

# --- 플러그인 설정의 핵심 부분 ---
# pluginlib가 우리 플러그인을 찾을 수 있도록 bt_plugins.xml 파일을 export합니다.
# 첫 번째 인자인 nav2_core는 플러그인 로더가 찾을 기본 클래스가 정의된 패키지입니다.
pluginlib_export_plugin_description_file(nav2_core bt_plugins.xml)

# --- 설치(install) 규칙 정의 ---
# 생성된 파일들을 colcon install 시 올바른 위치로 복사하도록 설정합니다.

# 1. 생성된 공유 라이브러리(.so 파일)를 lib 디렉토리에 설치합니다.
install(TARGETS
  my_nav2_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# 2. 다른 패키지에서 우리 헤더파일을 #include 할 수 있도록 include 디렉토리를 설치합니다.
install(
  DIRECTORY include/
  DESTINATION include)

# 3. 플러그인 정의 파일(bt_plugins.xml)을 share 디렉토리에 설치합니다.
install(
    FILES bt_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

# ament_cmake 패키지 빌드를 마무리하는 필수 명령입니다.
ament_package()