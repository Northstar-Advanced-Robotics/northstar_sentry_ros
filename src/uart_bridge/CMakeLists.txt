cmake_minimum_required(VERSION 3.8)
project(uart_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(uart_bridge
  "msg/AutoAim.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target uart_bridge "rosidl_typesupport_cpp")

add_executable(uart_bridge_node
  src/uart_bridge_node.cpp
  src/uart/messages/odometry_message.cpp
  src/uart/messages/health_message.cpp
  src/uart/messages/robot_id_message.cpp
  src/uart/messages/autoaim_message.cpp
  src/uart/messages/alive_message.cpp
  src/uart/messages/auto_path_message.cpp
  src/uart/handlers/odometry_handler.cpp
  src/uart/handlers/robot_id_handler.cpp
  src/uart/handlers/health_handler.cpp
  src/uart/messages/vision_localization_message.cpp
  src/uart/messages/fly_sky_meassage.cpp
  src/uart/ibus_reader.cpp
  src/uart/uart.cpp
  src/uart/crc/crc.cpp
)

target_compile_features(uart_bridge_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(uart_bridge_node PUBLIC src)

target_link_libraries(uart_bridge_node
  ${cpp_typesupport_target}
)
ament_target_dependencies(uart_bridge_node
  rclcpp
  std_msgs
  nav_msgs
  tf2_geometry_msgs
)

install(TARGETS uart_bridge_node
  DESTINATION lib/${PROJECT_NAME})

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
