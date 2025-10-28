cmake_minimum_required(VERSION 3.8)
project(jetson_csi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories (includes nlohmann/json.hpp header-only library)
include_directories(include)

# Add ROS2 node executable
add_executable(csi_camera_node src/csi_camera_node.cpp)

ament_target_dependencies(csi_camera_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

target_link_libraries(csi_camera_node
  ${OpenCV_LIBRARIES}
)

# Add standalone capture executable (non-ROS2)
add_executable(csi_capture_image_save src/csi_capture_image_save.cpp)

target_link_libraries(csi_capture_image_save
  ${OpenCV_LIBRARIES}
)

# Install executables
install(TARGETS
  csi_camera_node
  csi_capture_image_save
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
