cmake_minimum_required(VERSION 3.8)
project(robot_lab_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/recenter_rviz_window.py
    scripts/static_lab_scene_markers.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
