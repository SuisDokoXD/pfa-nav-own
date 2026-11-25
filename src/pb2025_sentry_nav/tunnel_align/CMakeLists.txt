cmake_minimum_required(VERSION 3.10)
project(tunnel_align)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(tunnel_align_core SHARED
  src/tunnel_geometry.cpp
)
target_include_directories(tunnel_align_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(tunnel_align_supervisor_node
  src/tunnel_align_supervisor_node.cpp
)
target_link_libraries(tunnel_align_supervisor_node tunnel_align_core)
ament_target_dependencies(tunnel_align_supervisor_node
  rclcpp
  std_msgs
  example_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(sim_cboard_emulator_node
  src/sim_cboard_emulator_node.cpp
)
target_link_libraries(sim_cboard_emulator_node tunnel_align_core)
ament_target_dependencies(sim_cboard_emulator_node
  rclcpp
  std_msgs
  example_interfaces
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  tunnel_align_core
  tunnel_align_supervisor_node
  sim_cboard_emulator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tunnel_geometry test/test_tunnel_geometry.cpp)
  if(TARGET test_tunnel_geometry)
    target_link_libraries(test_tunnel_geometry tunnel_align_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(tunnel_align_core)
ament_package()
