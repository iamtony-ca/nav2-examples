cmake_minimum_required(VERSION 3.8)
project(custom_inflation_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)

# Add source and include directories
include_directories(include)

# Create the plugin library
add_library(${PROJECT_NAME} SHARED
  src/inflation_layer.cpp
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "nav2_costmap_2d"
  "pluginlib"
  "nav_msgs"
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export the plugin description file
pluginlib_export_plugin_description_file(nav2_costmap_2d custom_plugins.xml)

# Export dependencies for downstream packages
ament_export_dependencies(
  "rclcpp"
  "nav2_costmap_2d"
  "pluginlib"
)

# Export include directories
ament_export_include_directories(include)

# Export libraries to downstream packages
ament_export_libraries(${PROJECT_NAME})

# Register the plugin library with ament
ament_package()