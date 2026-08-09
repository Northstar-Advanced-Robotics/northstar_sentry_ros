cmake_minimum_required(VERSION 3.10)
project(localization CXX C)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclc REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)

add_executable(cov_filter_node src/cov_filter_node.cpp)
target_link_libraries(cov_filter_node
PRIVATE
  rclcpp::rclcpp
  nav_msgs::nav_msgs
  apriltag_msgs::apriltag_msgs
)

add_executable(cov_filter_node_c src/cov_filter_node.c)
target_link_libraries(cov_filter_node_c
PRIVATE
  rclc::rclc
  nav_msgs::nav_msgs
  apriltag_msgs::apriltag_msgs
)

add_executable(tag_translator_node src/tag_translator.cpp)
target_link_libraries(tag_translator_node
PRIVATE
  rclcpp::rclcpp
  apriltag_msgs::apriltag_msgs
  isaac_ros_apriltag_interfaces::isaac_ros_apriltag_interfaces__rosidl_typesupport_cpp
)

add_executable(tag_translator_node_c src/tag_translator.c)
target_compile_features(tag_translator_node_c PRIVATE c_std_17)
target_link_libraries(tag_translator_node_c
PRIVATE
  rclc::rclc
  apriltag_msgs::apriltag_msgs
  isaac_ros_apriltag_interfaces::isaac_ros_apriltag_interfaces__rosidl_typesupport_c
)

install(TARGETS
  cov_filter_node
  cov_filter_node_c
  tag_translator_node
  tag_translator_node_c
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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
