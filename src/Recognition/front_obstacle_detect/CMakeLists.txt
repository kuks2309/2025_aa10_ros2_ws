cmake_minimum_required(VERSION 3.8)
project(front_obstacle_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(front_obstacle_detect_node
  src/front_obstacle_detect_node.cpp
  src/obstacle_detector.cpp
)

ament_target_dependencies(front_obstacle_detect_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  front_obstacle_detect_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
