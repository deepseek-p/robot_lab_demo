#include <chrono>
#include <cmath>
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
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{
using moveit::planning_interface::MoveGroupInterface;

// Geometry contract shared with lab_ur_gripper.urdf.xacro and lab_minimal.sdf:
//   - gripper_tcp sits 0.105 m beyond tool0 along tool z
//   - sample_block (0.07 m cube) rests at (0.58, 0.18), centre z ~0.845
//   - target_pad top surface is at z ~0.82 at (0.58, -0.18)
constexpr double kTcpOffset = 0.105;
constexpr double kBlockX = 0.58;
constexpr double kBlockY = 0.18;
constexpr double kBlockCenterZ = 0.845;
constexpr double kPadX = 0.58;
constexpr double kPadY = -0.18;
constexpr double kPlaceCenterZ = 0.865;
constexpr double kApproachClearance = 0.105;

constexpr double kGripperOpen = 0.0;
constexpr double kGripperClosedOnBlock = 0.011;  // jaw gap = 0.092 - 2*cmd

geometry_msgs::msg::Pose top_down_tool0_pose(double x, double y, double tcp_z)
{
  // Tool z points straight down: rotation of pi about world X.
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = tcp_z + kTcpOffset;
  return pose;
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

bool move_to_pose(
  rclcpp::Logger logger,
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & pose,
  const std::string & label)
{
  RCLCPP_INFO(logger, "Planning pose target: %s", label.c_str());
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(pose);

  MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group.plan(plan))) {
    RCLCPP_ERROR(logger, "Planning failed for pose target: %s", label.c_str());
    return false;
  }
  if (!static_cast<bool>(move_group.execute(plan))) {
    RCLCPP_ERROR(logger, "Execution failed for pose target: %s", label.c_str());
    return false;
  }
  return true;
}

bool move_linear(
  rclcpp::Logger logger,
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & pose,
  const std::string & label)
{
  RCLCPP_INFO(logger, "Cartesian move: %s", label.c_str());
  move_group.setStartStateToCurrentState();

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction =
    move_group.computeCartesianPath({pose}, 0.005, 0.0, trajectory);
  if (fraction > 0.95) {
    if (static_cast<bool>(move_group.execute(trajectory))) {
      return true;
    }
    RCLCPP_WARN(logger, "Cartesian execution failed for %s; retrying with OMPL", label.c_str());
  } else {
    RCLCPP_WARN(
      logger, "Cartesian path for %s only covered %.0f%%; falling back to OMPL",
      label.c_str(), fraction * 100.0);
  }
  return move_to_pose(logger, move_group, pose, label);
}

bool move_to_joints(
  rclcpp::Logger logger,
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & joints,
  const std::string & label)
{
  RCLCPP_INFO(logger, "Planning joint target: %s", label.c_str());
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(joints);

  MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group.plan(plan))) {
    RCLCPP_ERROR(logger, "Planning failed for joint target: %s", label.c_str());
    return false;
  }
  if (!static_cast<bool>(move_group.execute(plan))) {
    RCLCPP_ERROR(logger, "Execution failed for joint target: %s", label.c_str());
    return false;
  }
  return true;
}

class GripperIo
{
public:
  explicit GripperIo(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    finger_cmd_(node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/gripper_position_controller/commands", 10)),
    attach_block_(node->create_publisher<std_msgs::msg::Empty>(
        "/gripper/attach/sample_block", 10)),
    detach_block_(node->create_publisher<std_msgs::msg::Empty>(
        "/gripper/detach/sample_block", 10)),
    detach_vial_(node->create_publisher<std_msgs::msg::Empty>(
        "/gripper/detach/sample_vial_red", 10))
  {
  }

  void command_fingers(double per_finger_travel)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {per_finger_travel, per_finger_travel};
    finger_cmd_->publish(msg);
    rclcpp::sleep_for(std::chrono::milliseconds(900));
  }

  void attach_block()
  {
    publish_repeated(attach_block_);
  }

  void detach_all()
  {
    publish_repeated(detach_block_);
    publish_repeated(detach_vial_);
  }

private:
  void publish_repeated(const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr & pub)
  {
    std_msgs::msg::Empty msg;
    for (int i = 0; i < 3; ++i) {
      pub->publish(msg);
      rclcpp::sleep_for(std::chrono::milliseconds(150));
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr finger_cmd_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_block_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_block_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_vial_;
};

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
    "robot_lab_scripted_pick_demo",
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
    GripperIo gripper(node);

    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.15);
    move_group.setMaxAccelerationScalingFactor(0.15);

    RCLCPP_INFO(logger, "Using planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "Using end effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(logger, "Using controller: scaled_joint_trajectory_controller");
    add_lab_table_collision_object(planning_scene_interface, move_group.getPlanningFrame());

    // Give discovery a moment, then release any joint the DetachableJoint
    // plugin may have created at spawn time.
    rclcpp::sleep_for(std::chrono::seconds(1));
    gripper.detach_all();
    gripper.command_fingers(kGripperOpen);

    const std::map<std::string, double> ready_joints = {
      {"shoulder_pan_joint", 0.0},
      {"shoulder_lift_joint", -1.57},
      {"elbow_joint", 0.0},
      {"wrist_1_joint", -1.57},
      {"wrist_2_joint", 0.0},
      {"wrist_3_joint", 0.0},
    };

    // Deterministic approach configurations, solved once with move_group's
    // collision-aware /compute_ik for the top-down poses below (UR5e default
    // kinematics). Joint-space targets avoid per-run IK sampling flakiness:
    // random goal sampling occasionally lands in a folded-wrist family where
    // the fingers contact the forearm and planning fails. The two configs
    // share lift/elbow/wrist values, so the transfer is a pure base rotation.
    const std::map<std::string, double> pre_grasp_joints = {
      {"shoulder_pan_joint", 0.0796},
      {"shoulder_lift_joint", -1.2424},
      {"elbow_joint", 1.6772},
      {"wrist_1_joint", -2.0056},
      {"wrist_2_joint", -1.5708},
      {"wrist_3_joint", -1.4912},
    };
    const std::map<std::string, double> transfer_joints = {
      {"shoulder_pan_joint", -0.5222},
      {"shoulder_lift_joint", -1.2424},
      {"elbow_joint", 1.6772},
      {"wrist_1_joint", -2.0056},
      {"wrist_2_joint", -1.5708},
      {"wrist_3_joint", -2.0930},
    };

    const auto pre_grasp =
      top_down_tool0_pose(kBlockX, kBlockY, kBlockCenterZ + kApproachClearance);
    const auto grasp = top_down_tool0_pose(kBlockX, kBlockY, kBlockCenterZ + 0.005);
    const auto transfer =
      top_down_tool0_pose(kPadX, kPadY, kBlockCenterZ + kApproachClearance);
    const auto place = top_down_tool0_pose(kPadX, kPadY, kPlaceCenterZ);

    ok = ok && move_to_joints(logger, move_group, ready_joints, "ready");
    ok = ok && move_to_joints(logger, move_group, pre_grasp_joints, "pre_grasp");
    ok = ok && move_linear(logger, move_group, grasp, "descend_to_grasp");
    if (ok) {
      gripper.command_fingers(kGripperClosedOnBlock);
      gripper.attach_block();
      RCLCPP_INFO(logger, "Sample block grasped and attached.");
    }
    ok = ok && move_linear(logger, move_group, pre_grasp, "lift");
    ok = ok && move_to_joints(logger, move_group, transfer_joints, "transfer_to_pad");
    ok = ok && move_linear(logger, move_group, place, "descend_to_place");
    if (ok) {
      gripper.detach_all();
      gripper.command_fingers(kGripperOpen);
      RCLCPP_INFO(logger, "Sample block released on the target pad.");
    }
    ok = ok && move_linear(logger, move_group, transfer, "retreat");
    ok = ok && move_to_joints(logger, move_group, ready_joints, "return_ready");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Scripted pick demo failed: %s", ex.what());
    ok = false;
  }

  shutdown_node();

  if (!ok) {
    return 1;
  }

  RCLCPP_INFO(logger, "Scripted pick-and-place demo completed.");
  return 0;
}
