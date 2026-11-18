cmake_minimum_required(VERSION 3.8)
project(uatt_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
