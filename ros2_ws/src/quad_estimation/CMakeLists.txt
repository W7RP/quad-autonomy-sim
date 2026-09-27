cmake_minimum_required(VERSION 3.16)
project(quad_estimation LANGUAGES CXX)

# Default to an optimised build: a bare `colcon build` otherwise configures with
# no build type (-O0), which made the estimator's IMU callback ~27x slower (372 vs 13.5 us mean) and
# blew its real-time budget.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# ROS-free core: the filter math plus the per-sample pipeline around it
# (timeline, alignment, buffering). Fixed-size Eigen, no allocation.
add_library(eskf_core src/eskf.cpp src/pipeline.cpp src/params.cpp)
target_include_directories(eskf_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(eskf_core PUBLIC Eigen3::Eigen)

# The node. alloc_probe.cpp replaces global operator new, so it is linked into
# this executable only (never into a library other code might pick up).
add_executable(eskf_node src/eskf_node.cpp src/alloc_probe.cpp)
target_link_libraries(eskf_node eskf_core)
ament_target_dependencies(eskf_node rclcpp px4_msgs nav_msgs diagnostic_msgs)

# Offline replay of recorded flights through the same pipeline (tuning, regression).
add_executable(eskf_replay src/eskf_replay.cpp)
target_link_libraries(eskf_replay eskf_core)

install(TARGETS eskf_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS eskf_node eskf_replay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The filter source is compiled into the test directly (not linked from
  # eskf_core) so EIGEN_RUNTIME_NO_MALLOC is consistent across every TU that
  # instantiates Eigen code: the tests can then arm Eigen's malloc trap around
  # predict()/update_*() and prove they never allocate.
  ament_add_gtest(test_eskf test/test_eskf.cpp src/eskf.cpp)
  target_include_directories(test_eskf PRIVATE include)
  target_link_libraries(test_eskf Eigen3::Eigen)
  target_compile_definitions(test_eskf PRIVATE EIGEN_RUNTIME_NO_MALLOC)

  ament_add_gtest(test_support test/test_support.cpp)
  target_link_libraries(test_support eskf_core)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
