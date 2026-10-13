cmake_minimum_required(VERSION 3.13) 
project(path_planning CXX)
 
find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(path_planning_node
    src/main.cpp
)

target_compile_features(path_planning_node PRIVATE cxx_std_20)

target_link_libraries(path_planning_node
PRIVATE
    rclcpp::rclcpp 
    nav_msgs::nav_msgs 
    visualization_msgs::visualization_msgs 
    geometry_msgs::geometry_msgs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    Eigen3::Eigen
)
target_include_directories(path_planning_node
PRIVATE
  src
)
install(TARGETS
  path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
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
