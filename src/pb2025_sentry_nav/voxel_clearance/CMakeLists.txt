cmake_minimum_required(VERSION 3.10)
project(voxel_clearance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_library(voxel_clearance_core SHARED
  src/clearance_grid.cpp
)
target_include_directories(voxel_clearance_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(voxel_clearance_node
  src/voxel_clearance_node.cpp
)
target_include_directories(voxel_clearance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(voxel_clearance_node
  voxel_clearance_core
  ${PCL_LIBRARIES}
)
ament_target_dependencies(voxel_clearance_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
)

install(TARGETS voxel_clearance_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS voxel_clearance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_clearance_grid
    test/test_clearance_grid.cpp
  )
  target_link_libraries(test_clearance_grid
    voxel_clearance_core
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(voxel_clearance_core)
ament_package()
