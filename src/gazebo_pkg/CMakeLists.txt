cmake_minimum_required(VERSION 3.5)
project(gazebo_pkg)

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
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# gazebo_ros_template
add_library(gazebo_ros_template SHARED src/gazebo_ros_template.cpp)
target_include_directories(gazebo_ros_template PUBLIC include)



ament_target_dependencies(gazebo_ros_template
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_template)


# gazebo_ros_contact
add_library(gazebo_ros_contact SHARED src/gazebo_ros_contact.cpp)
target_include_directories(gazebo_ros_contact PUBLIC include)
target_link_libraries(gazebo_ros_contact ${GAZEBO_LIBRARIES})

ament_target_dependencies(gazebo_ros_contact
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_contact)

install(TARGETS
gazebo_ros_contact
ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
gazebo_ros_template
ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
