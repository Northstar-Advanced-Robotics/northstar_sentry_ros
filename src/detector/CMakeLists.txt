cmake_minimum_required(VERSION 3.10)
project(detector LANGUAGES CXX CUDA)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose build type" FORCE)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(OpenCV_DIR "/home/northstar/opencv/build") # NOTE: idk

# Find all required dependencies
find_package(CUDAToolkit REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)
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
find_package(uart_bridge REQUIRED)
find_package(message_filters REQUIRED)

set(PF_LIB_SOURCES
    # deps/robomaster-particle-filters/src/plate_orbit/particle_filter.cu
    detector_submodule/deps/robomaster-particle-filters/src/fast_plate_orbit/particle_filter.cu
    # deps/robomaster-particle-filters/src/plate_orbit_v2/particle_filter.cu
)

# 2. Create the library target
add_library(robomaster_particle_filters_lib SHARED ${PF_LIB_SOURCES})

set_target_properties(robomaster_particle_filters_lib PROPERTIES LINKER_LANGUAGE CXX)

target_compile_features(robomaster_particle_filters_lib PUBLIC cxx_std_20)

target_compile_options(robomaster_particle_filters_lib PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
        --extended-lambda
        --expt-relaxed-constexpr
    >
)

# 4. Manually apply all the necessary public properties to our new library target
target_include_directories(robomaster_particle_filters_lib PUBLIC
    detector_submodule/deps/robomaster-particle-filters/include
    detector_submodule/deps/robomaster-particle-filters/deps/eigen
    detector_submodule/deps/robomaster-particle-filters/deps/particle-filter
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    "${CUDAToolkit_INCLUDE_DIRS}/cccl"
)
target_compile_definitions(robomaster_particle_filters_lib PUBLIC
    $<$<COMPILE_LANGUAGE:CXX>:PF_TARGET_OMP THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP>
    $<$<COMPILE_LANGUAGE:CUDA>:PF_TARGET_CUDA THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA>
)
# --- Define your executable and list ALL source files ---
add_executable(detector
    src/ros_main.cpp
    # src/Northstar_CV2.cpp
    detector_submodule/src/detector.cpp
    detector_submodule/src/pnp_solver.cpp
    detector_submodule/src/number_classifier.cpp
    detector_submodule/src/uart/messages/autoaim_message.cpp
    detector_submodule/src/camera/usb_camera.cpp
    detector_submodule/src/camera/realsense_camera.cpp
)

# --- Link all dependencies to your executable ---

# Add include paths so you can use #include <...>
target_include_directories(detector PRIVATE
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/detector_submodule/src"
    "${CUDAToolkit_INCLUDE_DIRS}/cccl"
)

# Link the actual libraries
target_link_libraries(detector
    ${OpenCV_LIBS}
    realsense2::realsense2
    Threads::Threads
    robomaster_particle_filters_lib
    CUDA::toolkit
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

# cmake -DCMAKE_BUILD_TYPE=Debug ..
