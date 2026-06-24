#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
using moveit::planning_interface::MoveGroupInterface;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;

constexpr double kTcpOffset = 0.105;
constexpr double kApproach = 0.105;
constexpr double kGripperOpen = 0.0;

const std::vector<std::string> kArmJoints = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

// Known-good elbow-up, wrist-down configuration (solved offline with the
// collision-aware /compute_ik). Used as the IK seed so online solutions stay
// in the same family instead of sampling folded-wrist configurations where
// the fingers collide with the forearm.
const std::vector<double> kFamilySeed = {0.0796, -1.2424, 1.6772, -2.0056, -1.5708, -1.4912};

struct GraspSpec
{
  double grasp_tcp_z;     // TCP height when closing the fingers
  double close_command;   // per-finger travel that grips the object
  double place_tcp_z;     // TCP height when releasing at the target
};

// Geometry contract with lab_minimal.sdf (objects rest on the 0.81 bench).
const std::map<std::string, GraspSpec> kGraspSpecs = {
  {"sample_block", {0.850, 0.011, 0.865}},
  {"sample_vial_red", {0.885, 0.027, 0.890}},
};

const std::map<std::string, std::pair<double, double>> kPlaceTargets = {
  {"target_pad", {0.58, -0.18}},
  {"sample_tray", {0.45, -0.30}},
  {"bench_center", {0.62, 0.0}},
};

geometry_msgs::msg::Pose top_down_tool0_pose(double x, double y, double tcp_z)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 1.0;
  pose.orientation.w = 0.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = tcp_z + kTcpOffset;
  return pose;
}

std::string extract_json_string(const std::string & json, const std::string & key)
{
  const auto key_pos = json.find("\"" + key + "\"");
  if (key_pos == std::string::npos) {
    return "";
  }
  const auto colon = json.find(':', key_pos);
  const auto first_quote = json.find('"', colon + 1);
  const auto second_quote = json.find('"', first_quote + 1);
  if (first_quote == std::string::npos || second_quote == std::string::npos) {
    return "";
  }
  return json.substr(first_quote + 1, second_quote - first_quote - 1);
}

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode()
  : Node(
      "pick_place_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    finger_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_position_controller/commands", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/task/status", 10);

    for (const auto & [object_name, spec] : kGraspSpecs) {
      (void)spec;
      attach_pubs_[object_name] = create_publisher<std_msgs::msg::Empty>(
        "/gripper/attach/" + object_name, 10);
      detach_pubs_[object_name] = create_publisher<std_msgs::msg::Empty>(
        "/gripper/detach/" + object_name, 10);
      pose_subs_.push_back(
        create_subscription<geometry_msgs::msg::PoseStamped>(
          "/perception/" + object_name + "/pose", 10,
          [this, object_name](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::scoped_lock lock(pose_mutex_);
            object_poses_[object_name] = *msg;
          }));
    }

    command_sub_ = create_subscription<std_msgs::msg::String>(
      "/task/command", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::scoped_lock lock(command_mutex_);
        pending_command_ = msg->data;
      });

    ik_client_ = create_client<GetPositionIK>("/compute_ik");
  }

  void publish_status(const std::string & state, const std::string & detail)
  {
    std_msgs::msg::String msg;
    msg.data = "{\"state\": \"" + state + "\", \"detail\": \"" + detail + "\"}";
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "status: %s (%s)", state.c_str(), detail.c_str());
  }

  std::optional<std::string> take_command()
  {
    std::scoped_lock lock(command_mutex_);
    if (pending_command_.empty()) {
      return std::nullopt;
    }
    std::string cmd = pending_command_;
    pending_command_.clear();
    return cmd;
  }

  std::optional<geometry_msgs::msg::PoseStamped> latest_pose(const std::string & name)
  {
    std::scoped_lock lock(pose_mutex_);
    const auto it = object_poses_.find(name);
    if (it == object_poses_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  void command_fingers(double travel)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {travel, travel};
    finger_cmd_->publish(msg);
    rclcpp::sleep_for(std::chrono::milliseconds(900));
  }

  void publish_repeated(const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr & pub)
  {
    std_msgs::msg::Empty msg;
    for (int i = 0; i < 3; ++i) {
      pub->publish(msg);
      rclcpp::sleep_for(std::chrono::milliseconds(150));
    }
  }

  void attach(const std::string & name) {publish_repeated(attach_pubs_.at(name));}
  void detach(const std::string & name) {publish_repeated(detach_pubs_.at(name));}

  void detach_all()
  {
    for (const auto & [name, pub] : detach_pubs_) {
      (void)name;
      publish_repeated(pub);
    }
  }

  std::optional<std::map<std::string, double>> solve_ik(
    const geometry_msgs::msg::Pose & pose)
  {
    if (!ik_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "/compute_ik service unavailable");
      return std::nullopt;
    }
    auto request = std::make_shared<GetPositionIK::Request>();
    request->ik_request.group_name = "ur_manipulator";
    request->ik_request.avoid_collisions = true;
    request->ik_request.robot_state.joint_state.name = kArmJoints;
    request->ik_request.robot_state.joint_state.position = kFamilySeed;
    request->ik_request.pose_stamped.header.frame_id = "world";
    request->ik_request.pose_stamped.pose = pose;
    request->ik_request.timeout.sec = 3;

    auto future = ik_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(8)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "IK request timed out");
      return std::nullopt;
    }
    const auto response = future.get();
    if (response->error_code.val != 1) {
      RCLCPP_ERROR(get_logger(), "IK failed with code %d", response->error_code.val);
      return std::nullopt;
    }
    std::map<std::string, double> joints;
    for (size_t i = 0; i < response->solution.joint_state.name.size(); ++i) {
      const auto & joint_name = response->solution.joint_state.name[i];
      for (const auto & arm_joint : kArmJoints) {
        if (joint_name == arm_joint) {
          joints[joint_name] = response->solution.joint_state.position[i];
        }
      }
    }
    return joints.size() == kArmJoints.size() ? std::make_optional(joints) : std::nullopt;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr finger_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> attach_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> detach_pubs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;

  std::mutex pose_mutex_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> object_poses_;
  std::mutex command_mutex_;
  std::string pending_command_;
};

void add_lab_table(
  moveit::planning_interface::PlanningSceneInterface & psi,
  const std::string & frame)
{
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = frame;
  table.id = "lab_table";
  shape_msgs::msg::SolidPrimitive top;
  top.type = shape_msgs::msg::SolidPrimitive::BOX;
  top.dimensions = {1.0, 0.7, 0.06};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.75;
  pose.position.z = 0.78;
  table.primitives.push_back(top);
  table.primitive_poses.push_back(pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;
  psi.applyCollisionObjects({table});
}

bool move_to_joints(
  rclcpp::Logger logger, MoveGroupInterface & mg,
  const std::map<std::string, double> & joints, const std::string & label)
{
  RCLCPP_INFO(logger, "joint move: %s", label.c_str());
  mg.setStartStateToCurrentState();
  mg.setJointValueTarget(joints);
  MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(mg.plan(plan))) {
    RCLCPP_ERROR(logger, "planning failed: %s", label.c_str());
    return false;
  }
  return static_cast<bool>(mg.execute(plan));
}

bool move_linear(
  rclcpp::Logger logger, MoveGroupInterface & mg,
  const geometry_msgs::msg::Pose & pose, const std::string & label)
{
  RCLCPP_INFO(logger, "cartesian move: %s", label.c_str());
  mg.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = mg.computeCartesianPath({pose}, 0.005, 0.0, trajectory);
  if (fraction > 0.95 && static_cast<bool>(mg.execute(trajectory))) {
    return true;
  }
  RCLCPP_WARN(logger, "cartesian %s degraded (%.0f%%), OMPL fallback", label.c_str(),
    fraction * 100.0);
  mg.setStartStateToCurrentState();
  mg.setPoseTarget(pose);
  MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(mg.plan(plan))) {
    RCLCPP_ERROR(logger, "fallback planning failed: %s", label.c_str());
    return false;
  }
  return static_cast<bool>(mg.execute(plan));
}

const std::map<std::string, double> kReadyJoints = {
  {"shoulder_pan_joint", 0.0}, {"shoulder_lift_joint", -1.57}, {"elbow_joint", 0.0},
  {"wrist_1_joint", -1.57}, {"wrist_2_joint", 0.0}, {"wrist_3_joint", 0.0}};

// Retry wrapper: dynamic obstacles legitimately abort planning/execution
// (move_group invalidates in-flight trajectories when the scene changes).
// Waiting briefly and replanning converts those safety aborts into success
// once the obstacle has moved on.
constexpr int kMaxAttempts = 4;
constexpr auto kRetryBackoff = std::chrono::seconds(2);

template<typename StepFn>
bool with_retry(rclcpp::Logger logger, const std::string & label, StepFn && step)
{
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    if (step()) {
      return true;
    }
    RCLCPP_WARN(
      logger, "%s failed (attempt %d/%d); waiting for scene to clear",
      label.c_str(), attempt, kMaxAttempts);
    rclcpp::sleep_for(kRetryBackoff);
  }
  return false;
}

bool execute_pick_place(
  const std::shared_ptr<PickPlaceNode> & node,
  MoveGroupInterface & mg,
  const std::string & object_name,
  const std::string & target_name)
{
  const auto logger = node->get_logger();
  const auto spec_it = kGraspSpecs.find(object_name);
  const auto target_it = kPlaceTargets.find(target_name);
  if (spec_it == kGraspSpecs.end() || target_it == kPlaceTargets.end()) {
    node->publish_status("error", "unknown object or target");
    return false;
  }
  const auto pose_opt = node->latest_pose(object_name);
  if (!pose_opt) {
    node->publish_status("error", "no perception pose for " + object_name);
    return false;
  }
  const double ox = pose_opt->pose.position.x;
  const double oy = pose_opt->pose.position.y;
  const auto [tx, ty] = target_it->second;
  const auto & spec = spec_it->second;
  RCLCPP_INFO(
    logger, "pick %s at (%.3f, %.3f) -> %s (%.3f, %.3f)",
    object_name.c_str(), ox, oy, target_name.c_str(), tx, ty);

  const auto pre_grasp = top_down_tool0_pose(ox, oy, spec.grasp_tcp_z + kApproach);
  const auto grasp = top_down_tool0_pose(ox, oy, spec.grasp_tcp_z);
  const auto pre_place = top_down_tool0_pose(tx, ty, spec.place_tcp_z + kApproach);
  const auto place = top_down_tool0_pose(tx, ty, spec.place_tcp_z);

  node->publish_status("executing", "pick " + object_name);
  node->detach_all();
  node->command_fingers(kGripperOpen);

  // IK is re-solved inside the retry loop: when the obstacle blocks an
  // approach pose, a later attempt finds a collision-free solution.
  bool ok = with_retry(logger, "approach_" + object_name, [&]() {
      const auto joints = node->solve_ik(pre_grasp);
      return joints && move_to_joints(logger, mg, *joints, "pre_grasp");
    });
  ok = ok && with_retry(logger, "descend_to_grasp", [&]() {
      return move_linear(logger, mg, grasp, "descend_to_grasp");
    });
  if (ok) {
    node->command_fingers(spec.close_command);
    node->attach(object_name);
  }
  ok = ok && with_retry(logger, "lift", [&]() {
      return move_linear(logger, mg, pre_grasp, "lift");
    });
  ok = ok && with_retry(logger, "transfer_" + target_name, [&]() {
      const auto joints = node->solve_ik(pre_place);
      return joints && move_to_joints(logger, mg, *joints, "transfer");
    });
  ok = ok && with_retry(logger, "descend_to_place", [&]() {
      return move_linear(logger, mg, place, "descend_to_place");
    });
  if (ok) {
    node->detach(object_name);
    node->command_fingers(kGripperOpen);
  }
  ok = ok && with_retry(logger, "retreat", [&]() {
      return move_linear(logger, mg, pre_place, "retreat");
    });
  ok = ok && with_retry(logger, "return_ready", [&]() {
      return move_to_joints(logger, mg, kReadyJoints, "return_ready");
    });

  node->publish_status(ok ? "done" : "error", object_name + " -> " + target_name);
  return ok;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  const auto logger = node->get_logger();
  MoveGroupInterface mg(node, "ur_manipulator");
  moveit::planning_interface::PlanningSceneInterface psi;
  mg.setPlanningTime(10.0);
  mg.setNumPlanningAttempts(10);
  mg.setMaxVelocityScalingFactor(0.10);
  mg.setMaxAccelerationScalingFactor(0.10);
  add_lab_table(psi, mg.getPlanningFrame());

  node->publish_status("ready", "waiting for /task/command");
  RCLCPP_INFO(
    logger,
    "pick_place_node ready; send {\"action\": \"pick_place\", \"object\": \"sample_block\","
    " \"target\": \"target_pad\"} on /task/command");

  while (rclcpp::ok()) {
    const auto cmd = node->take_command();
    if (!cmd) {
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    const std::string action = extract_json_string(*cmd, "action");
    if (action == "pick_place") {
      const std::string object_name = extract_json_string(*cmd, "object");
      const std::string target_name = extract_json_string(*cmd, "target");
      execute_pick_place(node, mg, object_name, target_name);
    } else if (action == "home") {
      move_to_joints(logger, mg, kReadyJoints, "home");
      node->publish_status("done", "home");
    } else {
      node->publish_status("error", "unknown action: " + action);
    }
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
