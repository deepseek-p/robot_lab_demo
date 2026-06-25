cmake_minimum_required(VERSION 3.8)
project(robot_lab_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(preset_joint_demo src/preset_joint_demo.cpp)
target_compile_features(preset_joint_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  preset_joint_demo
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
)

add_executable(scripted_pick_demo src/scripted_pick_demo.cpp)
target_compile_features(scripted_pick_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  scripted_pick_demo
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
)

add_executable(pick_place_node src/pick_place_node.cpp)
target_compile_features(pick_place_node PUBLIC cxx_std_17)
ament_target_dependencies(
  pick_place_node
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
)

install(
  TARGETS preset_joint_demo scripted_pick_demo pick_place_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/task_orchestrator.py
    scripts/obstacle_monitor.py
    scripts/planning_scene_object_publisher.py
    scripts/evaluate_pick_place.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
