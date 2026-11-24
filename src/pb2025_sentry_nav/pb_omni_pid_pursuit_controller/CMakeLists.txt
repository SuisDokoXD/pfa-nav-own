cmake_minimum_required(VERSION 3.10)
project(pb_omni_pid_pursuit_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
  src/pursuit_geometry.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pursuit_geometry
    test/test_pursuit_geometry.cpp
    src/pursuit_geometry.cpp
  )
  target_include_directories(test_pursuit_geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

pluginlib_export_plugin_description_file(nav2_core pb_omni_pid_pursuit_controller.xml)

ament_auto_package()
