cmake_minimum_required(VERSION 3.5)
project(diff_to_ackermann)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(diff_to_ackermann src/diff_to_ackermann_node.cpp)
ament_target_dependencies(diff_to_ackermann rclcpp geometry_msgs)

install(TARGETS
  diff_to_ackermann
  DESTINATION lib/${PROJECT_NAME})

ament_package()