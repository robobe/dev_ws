cmake_minimum_required(VERSION 3.5)
project(diffbot_gazebo)

# find dependencies
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
