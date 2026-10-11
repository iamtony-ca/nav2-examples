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
find_package(robot_interfaces REQUIRED)  
find_package(rclcpp_lifecycle REQUIRED)


# 라이브러리 타겟
add_library(${PROJECT_NAME} SHARED
  src/agent_layer.cpp
  src/agent_mask_canvas.cpp   # [10-10 W1-2] 칠한 사각형만 지우기 / 바뀐 사각형만 publish
)
# [10-10 W1-2] 새 파일은 경고를 켜서 빌드한다 (원래 파일의 빌드 옵션은 그대로).
set_source_files_properties(src/agent_mask_canvas.cpp PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic")
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle nav2_costmap_2d pluginlib geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs robot_interfaces
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

# [10-10 W1-2] AgentMaskCanvas 동등성 시험 (원본 memset 방식과 매 사이클 비교). ROS node 를 만들지 않는다.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 플러그인 .so 에 링크하지 않고 소스를 직접 넣는다: underlay(v2_ws) 의 같은 이름 .so 가 LD_LIBRARY_PATH 에서 먼저 잡힌다.
  ament_add_gtest(test_agent_mask_canvas
    test/test_agent_mask_canvas.cpp
    src/agent_mask_canvas.cpp
    TIMEOUT 300
  )
  if(TARGET test_agent_mask_canvas)
    find_package(nav2_msgs REQUIRED)   # [10-10 W1-2][리뷰] fork publisher 모형의 Costmap / CostmapUpdate
    target_include_directories(test_agent_mask_canvas PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(test_agent_mask_canvas rclcpp nav2_costmap_2d nav2_msgs geometry_msgs tf2 tf2_geometry_msgs)
    target_compile_options(test_agent_mask_canvas PRIVATE -Wall -Wextra)
    # [10-10 W1-2][리뷰] 같은 src/ 아래에 replan_monitor 가 있으면 (pv_w2_ws) path_validator 의 실제 CostmapMirror 를 넣어
    #   AgentLayer → publisher → mirror 끝에서 끝까지 본다. amhs 처럼 위치가 다르면 REPLAN_MONITOR_SRC_DIR 로 넘기거나
    #   그 부분만 빠진다 (나머지 시험은 그대로).
    set(REPLAN_MONITOR_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../replan_monitor" CACHE PATH "replan_monitor 소스 (시험용)")
    if(EXISTS "${REPLAN_MONITOR_SRC_DIR}/src/costmap_mirror.cpp")
      target_sources(test_agent_mask_canvas PRIVATE "${REPLAN_MONITOR_SRC_DIR}/src/costmap_mirror.cpp")
      target_include_directories(test_agent_mask_canvas PRIVATE "${REPLAN_MONITOR_SRC_DIR}/include")
      target_compile_definitions(test_agent_mask_canvas PRIVATE AGENT_MASK_TEST_WITH_MIRROR)
      message(STATUS "[10-10 W1-2] test_agent_mask_canvas: CostmapMirror 포함 (${REPLAN_MONITOR_SRC_DIR})")
    else()
      message(STATUS "[10-10 W1-2] test_agent_mask_canvas: replan_monitor 소스 없음 — mirror 끝단 시험 생략")
    endif()
  endif()
endif()

# export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp rclcpp_lifecycle nav2_costmap_2d pluginlib geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs robot_interfaces
)

ament_package()
