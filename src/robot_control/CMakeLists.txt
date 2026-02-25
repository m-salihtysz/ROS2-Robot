cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

# Keyboard teleop node
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop
  rclcpp
  geometry_msgs
)

# Line follower node (şerit takibi - /line_error robot_interfaces/LineError)
add_executable(line_follower src/line_follower.cpp)
ament_target_dependencies(line_follower
  rclcpp
  geometry_msgs
  robot_interfaces
)

install(TARGETS
  keyboard_teleop
  line_follower
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
