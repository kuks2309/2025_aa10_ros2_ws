cmake_minimum_required(VERSION 3.8)
project(slam_toolbox_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Add executables
add_executable(slam_manager src/slam_manager.cpp)
add_executable(slam_cmd src/slam_cmd.cpp)

# Dependencies
ament_target_dependencies(slam_manager rclcpp std_msgs std_srvs)
ament_target_dependencies(slam_cmd rclcpp std_msgs)

# Install executables
install(TARGETS
  slam_manager
  slam_cmd
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()