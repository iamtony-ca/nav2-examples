cmake_minimum_required(VERSION 3.5)
# [변경 1] 프로젝트 이름 변경
project(regulated_graceful_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_stable_stopped_goal_checker REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
  nav_2d_utils
  angles
  nav2_stable_stopped_goal_checker
)

# [변경 2] 내부 라이브러리 타겟 이름을 변경 (충돌 방지 핵심!)
# 기존: add_library(smooth_control_law SHARED src/smooth_control_law.cpp)
add_library(regulated_smooth_control_law SHARED src/smooth_control_law.cpp)

# [변경 3] 의존성 타겟 이름도 변경된 이름으로 적용
ament_target_dependencies(regulated_smooth_control_law ${dependencies})

# Add Graceful Controller
# [변경 4] 메인 라이브러리 이름 변수 설정
set(library_name regulated_graceful_controller)

add_library(${library_name} SHARED
  src/graceful_controller.cpp
  src/parameter_handler.cpp
  src/path_handler.cpp
  src/utils.cpp
)

# [변경 5] 링크할 라이브러리 이름을 위에서 변경한 이름으로 수정
# 기존: target_link_libraries(${library_name} smooth_control_law)
target_link_libraries(${library_name} regulated_smooth_control_law)

ament_target_dependencies(${library_name} ${dependencies})

# [변경 6] 인스톨 타겟 이름 수정
install(TARGETS regulated_smooth_control_law ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)

# [변경 7] export 라이브러리 이름 수정
ament_export_libraries(regulated_smooth_control_law ${library_name})
ament_export_dependencies(${dependencies})

# [변경 8] 플러그인 XML 파일명 변경 (파일도 실제로 이름 바꿨는지 확인 필요)
pluginlib_export_plugin_description_file(nav2_core regulated_graceful_controller_plugin.xml)

ament_package()
