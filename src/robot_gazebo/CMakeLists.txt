cmake_minimum_required(VERSION 3.8)
project(robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts (sahte sensörler vb.)
install(PROGRAMS
  scripts/fake_sensors.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds directory
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz configs
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
