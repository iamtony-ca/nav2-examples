cmake_minimum_required(VERSION 3.8)
project(my_package)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

# Executable
add_executable(my_node_exec
  src/main.cpp
  src/my_node.cpp
)

# Header include paths
target_include_directories(my_node_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# C++ feature requirement
target_compile_features(my_node_exec PUBLIC cxx_std_17)

# Link dependencies
ament_target_dependencies(my_node_exec
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  example_interfaces
)

# Install executable
install(TARGETS my_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  example_interfaces
)

# Linting and tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
