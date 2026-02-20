cmake_minimum_required(VERSION 3.8)
project(arm_ik)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(typing_interfaces REQUIRED)

add_executable(arm_node src/main.cpp)

ament_target_dependencies(arm_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  typing_interfaces
)

install(TARGETS
  arm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
