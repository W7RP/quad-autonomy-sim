cmake_minimum_required(VERSION 3.16)
project(quad_offboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# ROS-free control logic, kept as its own library so it can be unit-tested
# without a simulator and reused by later phases.
add_library(waypoint_follower src/waypoint_follower.cpp)
target_include_directories(waypoint_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(offboard_square src/offboard_square_node.cpp)
target_link_libraries(offboard_square waypoint_follower)
ament_target_dependencies(offboard_square rclcpp px4_msgs)

install(TARGETS waypoint_follower EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS offboard_square DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_follower test/test_waypoint_follower.cpp)
  target_link_libraries(test_waypoint_follower waypoint_follower)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp px4_msgs)
ament_package()
