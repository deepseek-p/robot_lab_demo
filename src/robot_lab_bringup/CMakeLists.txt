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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(project_structure ../../tests/test_project_structure.py)
  ament_add_pytest_test(gazebo_classic_contracts ../../tests/test_gazebo_classic_contracts.py)
endif()

ament_package()
