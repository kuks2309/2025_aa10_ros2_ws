cmake_minimum_required(VERSION 3.8)
project(aa10_lidar_cone_maze)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(aa10_cone_navigation src/cone_navigation.cpp)
target_include_directories(aa10_cone_navigation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(aa10_cone_navigation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  aa10_cone_navigation
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "cv_bridge"
  "OpenCV"
  "nav_msgs"
)

install(TARGETS aa10_cone_navigation
  DESTINATION lib/${PROJECT_NAME})

# Install config and launch files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
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
