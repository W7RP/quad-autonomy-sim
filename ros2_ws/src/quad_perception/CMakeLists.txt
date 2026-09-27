cmake_minimum_required(VERSION 3.16)
project(quad_perception LANGUAGES CXX)

# Default to an optimised build (see quad_estimation/CMakeLists.txt for why).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(quad_estimation REQUIRED)  # frames.hpp (NED/FRD <-> ENU/FLU)

add_executable(px4_odometry_bridge src/px4_odometry_bridge.cpp)
target_include_directories(px4_odometry_bridge PRIVATE include)
target_link_libraries(px4_odometry_bridge quad_estimation::eskf_core)
ament_target_dependencies(px4_odometry_bridge rclcpp px4_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS px4_odometry_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odometry_conversion test/test_odometry_conversion.cpp)
  target_include_directories(test_odometry_conversion PRIVATE include)
  target_link_libraries(test_odometry_conversion quad_estimation::eskf_core)
endif()

ament_package()
