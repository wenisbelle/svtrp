cmake_minimum_required(VERSION 3.8)
project(kalman_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2_ros REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include_directories(${EIGEN_INCLUDE_DIRS})


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

add_executable(kalman_filter_velocity src/kalman_filter_velocity.cpp)
ament_target_dependencies(kalman_filter_velocity rclcpp nav_msgs sensor_msgs Eigen3)

add_executable(publish_odom_frame_filtered src/publish_odom_frame_filtered.cpp)
ament_target_dependencies(publish_odom_frame_filtered rclcpp nav_msgs tf2_ros)


install(TARGETS
  kalman_filter_velocity 
  publish_odom_frame_filtered
  DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  params
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
