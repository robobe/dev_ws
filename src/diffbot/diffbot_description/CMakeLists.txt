cmake_minimum_required(VERSION 3.5)
project(diffbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo ros2_control urdf launch meshes models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
