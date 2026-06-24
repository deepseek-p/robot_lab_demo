#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
using moveit::planning_interface::MoveGroupInterface;

struct JointTarget
{
  std::string name;
  std::map<std::string, double> joints;
};

std::map<std::string, double> ur_joint_target(
  double shoulder_pan,
  double shoulder_lift,
  double elbow,
  double wrist_1,
  double wrist_2,
  double wrist_3)
{
  return {
    {"shoulder_pan_joint", shoulder_pan},
    {"shoulder_lift_joint", shoulder_lift},
    {"elbow_joint", elbow},
    {"wrist_1_joint", wrist_1},
    {"wrist_2_joint", wrist_2},
    {"wrist_3_joint", wrist_3},
  };
}

void add_lab_table_collision_object(
  moveit::planning_interface::PlanningSceneInterface & planning_scene_interface,
  const std::string & planning_frame)
{
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = planning_frame;
  table.id = "lab_table";

  shape_msgs::msg::SolidPrimitive top;
  top.type = shape_msgs::msg::SolidPrimitive::BOX;
  top.dimensions = {1.0, 0.7, 0.06};

  geometry_msgs::msg::Pose top_pose;
  top_pose.orientation.w = 1.0;
  top_pose.position.x = 0.75;
  top_pose.position.y = 0.0;
  top_pose.position.z = 0.78;

  table.primitives.push_back(top);
  table.primitive_poses.push_back(top_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface.applyCollisionObjects({table});
  rclcpp::sleep_for(std::chrono::milliseconds(500));
}

bool plan_and_execute(
  rclcpp::Logger logger,
  MoveGroupInterface & move_group,
  const JointTarget & target)
{
  RCLCPP_INFO(logger, "Planning target: %s", target.name.c_str());
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(target.joints);

  MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(move_group.plan(plan));
  if (!planned) {
    RCLCPP_ERROR(logger, "Planning failed for target: %s", target.name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Executing target: %s", target.name.c_str());
  const bool executed = static_cast<bool>(move_group.execute(plan));
  if (!executed) {
    RCLCPP_ERROR(logger, "Execution failed for target: %s", target.name.c_str());
    return false;
  }

  return true;
}

std::string join_topics(const std::vector<std::string> & topics)
{
  std::string result;
  for (const auto & topic : topics) {
    if (!result.empty()) {
      result += ", ";
    }
    result += topic;
  }
  return result;
}

bool wait_for_required_bringup_topics(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::Logger logger,
  std::chrono::seconds timeout)
{
  const std::vector<std::string> required_topics = {"/robot_description", "/joint_states"};
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    bool ready = true;
    for (const auto & topic : required_topics) {
      if (node->count_publishers(topic) == 0) {
        ready = false;
        break;
      }
    }

    if (ready) {
      return true;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  std::vector<std::string> missing_topics;
  for (const auto & topic : required_topics) {
    if (node->count_publishers(topic) == 0) {
      missing_topics.push_back(topic);
    }
  }

  RCLCPP_ERROR(
    logger,
    "Start the demo bringup first; missing required topic publisher(s): %s",
    join_topics(missing_topics).c_str());
  RCLCPP_ERROR(logger, "  ros2 launch robot_lab_bringup lab_ur_moveit_gz.launch.py");
  RCLCPP_ERROR(logger, "Then run this node in a second terminal, or use:");
  RCLCPP_ERROR(
    logger,
    "  ros2 launch robot_lab_bringup lab_ur_moveit_gz.launch.py auto_start_task:=true");
  return false;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "robot_lab_preset_joint_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const auto logger = node->get_logger();
  bool ok = true;

  auto shutdown_node = [&executor, &spinner]() {
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
  };

  if (!wait_for_required_bringup_topics(node, logger, std::chrono::seconds(10))) {
    shutdown_node();
    return 2;
  }

  try {
    MoveGroupInterface move_group(node, "ur_manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.04);
    move_group.setMaxAccelerationScalingFactor(0.04);

    RCLCPP_INFO(logger, "Using planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "Using controller: scaled_joint_trajectory_controller");
    add_lab_table_collision_object(planning_scene_interface, move_group.getPlanningFrame());

    const std::vector<JointTarget> targets = {
      {"ready", ur_joint_target(0.0, -1.57, 0.0, -1.57, 0.0, 0.0)},
      {"approach_sample", ur_joint_target(-0.12, -1.48, 0.30, -1.72, -0.20, 0.0)},
      {"pickup_pose_like", ur_joint_target(-0.12, -1.47, 0.28, -1.72, -0.18, 0.0)},
      {"place_pose_like", ur_joint_target(0.12, -1.47, 0.28, -1.72, -0.18, 0.0)},
      {"return_ready", ur_joint_target(0.0, -1.57, 0.0, -1.57, 0.0, 0.0)},
    };

    for (const auto & target : targets) {
      if (!plan_and_execute(logger, move_group, target)) {
        ok = false;
        break;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Preset joint demo failed: %s", ex.what());
    ok = false;
  }

  shutdown_node();

  if (!ok) {
    return 1;
  }

  RCLCPP_INFO(logger, "Preset joint demo completed.");
  return 0;
}
