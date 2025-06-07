cmake_minimum_required(VERSION 3.10)
project(smart_recovery)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########## Dependencies ######################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)        # msgs / srv / action
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

########## Targets ###########################################################
# 1) SmartRecoveryServer
add_executable(smart_recovery_server
  src/smart_recovery_server.cpp
)
ament_target_dependencies(smart_recovery_server
  rclcpp rclcpp_action nav_msgs geometry_msgs std_msgs tf2_ros
  robot_interfaces
)
install(TARGETS smart_recovery_server
  DESTINATION lib/${PROJECT_NAME})

# 2) SmartRecoveryPlanner
add_executable(smart_recovery_planner
  src/smart_recovery_planner.cpp
)
ament_target_dependencies(smart_recovery_planner
  rclcpp nav_msgs geometry_msgs nav_msgs nav2_msgs tf2_ros tf2_geometry_msgs
  robot_interfaces
)
install(TARGETS smart_recovery_planner
  DESTINATION lib/${PROJECT_NAME})

# 3) MPPI CmdVel Generator
add_executable(mppi_cmdvel_generator
  src/MPPICmdVelGenerator.cpp
)
ament_target_dependencies(mppi_cmdvel_generator
  rclcpp nav_msgs geometry_msgs nav_msgs nav2_msgs tf2_ros tf2_geometry_msgs
  std_msgs robot_interfaces angles
)
install(TARGETS mppi_cmdvel_generator
  DESTINATION lib/${PROJECT_NAME})

########## Launch directory ##################################################
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

########## Export ############################################################
ament_package()
