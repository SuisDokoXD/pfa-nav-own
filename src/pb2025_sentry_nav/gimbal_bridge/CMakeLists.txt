cmake_minimum_required(VERSION 3.8)
project(gimbal_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(gimbal_bridge_node src/gimbal_bridge_node.cpp)
target_include_directories(gimbal_bridge_node PRIVATE include)
ament_target_dependencies(gimbal_bridge_node
  rclcpp geometry_msgs std_msgs sensor_msgs)

install(TARGETS gimbal_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
