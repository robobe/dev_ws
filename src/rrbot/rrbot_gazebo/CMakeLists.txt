cmake_minimum_required(VERSION 3.5)
project(rrbot_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

add_library(simple_world_plugin SHARED src/simple_world_plugin.cpp)

target_include_directories(simple_world_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(simple_world_plugin
  "gazebo_ros"
  "rclcpp"
  )


install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
simple_world_plugin
  DESTINATION share/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
