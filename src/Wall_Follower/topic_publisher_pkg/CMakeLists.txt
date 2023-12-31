cmake_minimum_required(VERSION 3.5)
project(topic_publisher_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_messages REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(move_robot_node src/move_robot.cpp)
ament_target_dependencies(move_robot_node rclcpp geometry_msgs sensor_msgs)

#add_executable(move_robot_service_node src/move_robot_service_inloop.cpp)
#ament_target_dependencies(move_robot_service_node rclcpp geometry_msgs sensor_msgs custom_messages)

#add_executable(move_robot_service_action_node src/move_robot_service_action.cpp)
#ament_target_dependencies(move_robot_service_action_node rclcpp rclcpp_action geometry_msgs sensor_msgs custom_messages)

add_executable(move_robot_service_action_node_v1 src/move_robot_service_action_v1.cpp)
ament_target_dependencies(move_robot_service_action_node_v1 rclcpp rclcpp_action geometry_msgs sensor_msgs custom_messages)

install(TARGETS
   move_robot_node
   #move_robot_service_node
   #move_robot_service_action_node
   move_robot_service_action_node_v1
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
