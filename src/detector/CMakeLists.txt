cmake_minimum_required(VERSION 3.10)
project(detector LANGUAGES CXX CUDA)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose build type" FORCE)
endif()

# Tell the compiler this is for the Jetson Orin AGX (Ampere)
set(CMAKE_CUDA_ARCHITECTURES 87)

find_package(CUDAToolkit REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)

find_package(uart_bridge REQUIRED)

set(PF_LIB_SOURCES
    # deps/robomaster-particle-filters/src/plate_orbit/particle_filter.cu
    detector_submodule/deps/robomaster-particle-filters/src/fast_plate_orbit/particle_filter.cu
    # deps/robomaster-particle-filters/src/plate_orbit_v2/particle_filter.cu
)

add_library(robomaster_particle_filters_lib SHARED ${PF_LIB_SOURCES})
target_compile_features(robomaster_particle_filters_lib PUBLIC cxx_std_20 cuda_std_20)
target_compile_options(robomaster_particle_filters_lib PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
        --extended-lambda
        --expt-relaxed-constexpr
    >
)

target_include_directories(robomaster_particle_filters_lib
PUBLIC
    detector_submodule/deps/robomaster-particle-filters/include
    detector_submodule/deps/robomaster-particle-filters/deps/particle-filter
PRIVATE
    detector_submodule/deps/robomaster-particle-filters/deps/eigen
)

target_link_libraries(robomaster_particle_filters_lib
PUBLIC
    CUDA::toolkit
)

target_compile_definitions(robomaster_particle_filters_lib PUBLIC
    $<$<COMPILE_LANGUAGE:CXX>:PF_TARGET_OMP THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP>
    $<$<COMPILE_LANGUAGE:CUDA>:PF_TARGET_CUDA THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA>
)

add_executable(detector
    src/ros_main.cpp
    detector_submodule/src/detector.cpp
    detector_submodule/src/pnp_solver.cpp
    detector_submodule/src/number_classifier.cpp
    detector_submodule/src/uart/messages/autoaim_message.cpp
    detector_submodule/src/camera/usb_camera.cpp
    detector_submodule/src/camera/realsense_camera.cpp
)

target_include_directories(detector
PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/detector_submodule/src"
)

target_link_libraries(detector
    ${OpenCV_LIBS}
    realsense2::realsense2
    Threads::Threads
    robomaster_particle_filters_lib
)

ament_target_dependencies(detector
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  tf2_geometry_msgs
  tf2
  tf2_ros
  uart_bridge
  message_filters
)

install(TARGETS
  detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
