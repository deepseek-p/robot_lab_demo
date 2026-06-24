cmake_minimum_required(VERSION 3.8)
project(robot_lab_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY srdf urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
