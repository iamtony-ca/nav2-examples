cmake_minimum_required(VERSION 3.16)
project(ammr_costmap_diff_publisher)

# [10-10 W1-D] global costmap diff publisher layer

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
)

# diff 엔진 (ROS 노드 없이 gtest 로 돌릴 수 있게 layer 와 분리)
add_library(${PROJECT_NAME}_engine SHARED
  src/diff_engine.cpp
)
target_include_directories(${PROJECT_NAME}_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}_engine nav2_costmap_2d nav2_msgs)

# pluginlib layer (library 이름 = plugin xml 의 path)
add_library(${PROJECT_NAME} SHARED
  src/diff_publisher_layer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_engine)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

pluginlib_export_plugin_description_file(nav2_costmap_2d diff_publisher_layer.xml)

# [10-10 W1-5] FastClearLayer — 따로 된 .so (diff publisher .so·xml 은 pv_w4 와 byte 동일하게 두고, 둘 중 하나만 써도 되게)
add_library(ammr_costmap_fast_clear SHARED
  src/fast_clear_layer.cpp
)
target_include_directories(ammr_costmap_fast_clear PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(ammr_costmap_fast_clear ${dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d fast_clear_layer.xml)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_engine ammr_costmap_fast_clear
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(FILES diff_publisher_layer.xml fast_clear_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 엔진 단위 테스트 (ROS 통신 없음)
  ament_add_gtest(test_diff_engine
    test/test_diff_engine.cpp
    TIMEOUT 600
  )
  if(TARGET test_diff_engine)
    target_link_libraries(test_diff_engine ${PROJECT_NAME}_engine)
    ament_target_dependencies(test_diff_engine nav2_costmap_2d nav2_msgs)
  endif()
  # layer + 실제 nav2_costmap_2d::CostmapSubscriber 연결 확인 (격리 domain 229, localhost)
  ament_add_gtest(test_diff_publisher_layer_ros
    test/test_diff_publisher_layer_ros.cpp
    TIMEOUT 120
  )
  if(TARGET test_diff_publisher_layer_ros)
    target_link_libraries(test_diff_publisher_layer_ros ${PROJECT_NAME})
    ament_target_dependencies(test_diff_publisher_layer_ros ${dependencies})
  endif()
  # [10-10 W1-5] FastClearLayer: 표준 resetLayers 와 master byte 동일·창 밖 불변·시간·동시성 (격리 domain 229, localhost)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  ament_add_gtest(test_fast_clear_layer
    test/test_fast_clear_layer.cpp
    TIMEOUT 900
  )
  if(TARGET test_fast_clear_layer)
    target_link_libraries(test_fast_clear_layer ammr_costmap_fast_clear)
    ament_target_dependencies(test_fast_clear_layer ${dependencies} nav_msgs sensor_msgs geometry_msgs tf2_ros)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_engine ammr_costmap_fast_clear)
ament_export_dependencies(${dependencies})
ament_package()
