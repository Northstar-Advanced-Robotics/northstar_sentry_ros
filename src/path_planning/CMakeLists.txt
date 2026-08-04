cmake_minimum_required(VERSION 3.13) 
project(path_planning)
 
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(path_planning_node
    src/main.cpp
    auto_drive/path_outline.hpp)

target_compile_features(path_planning_node PUBLIC cxx_std_20)

ament_target_dependencies(path_planning_node 
    rclcpp 
    nav_msgs 
    visualization_msgs 
    geometry_msgs
    tf2
    tf2_geometry_msgs
)

target_include_directories(path_planning_node PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIRS})

install(TARGETS path_planning_node DESTINATION lib/${PROJECT_NAME})

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
