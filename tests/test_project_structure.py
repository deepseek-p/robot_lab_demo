from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_expected_packages_and_demo_files_exist():
    expected_paths = [
        "src/robot_lab_description/package.xml",
        "src/robot_lab_description/CMakeLists.txt",
        "src/robot_lab_description/worlds/lab_minimal.sdf",
        "src/robot_lab_description/urdf/lab_ur_gripper.urdf.xacro",
        "src/robot_lab_description/urdf/inc/parallel_gripper_macro.xacro",
        "src/robot_lab_description/srdf/lab_ur_gripper.srdf.xacro",
        "src/robot_lab_bringup/package.xml",
        "src/robot_lab_bringup/CMakeLists.txt",
        "src/robot_lab_bringup/config/robot_lab_moveit.rviz",
        "src/robot_lab_bringup/config/lab_ur_controllers.yaml",
        "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py",
        "src/robot_lab_bringup/scripts/recenter_rviz_window.py",
        "src/robot_lab_tasks/package.xml",
        "src/robot_lab_tasks/CMakeLists.txt",
        "src/robot_lab_tasks/src/preset_joint_demo.cpp",
        "src/robot_lab_tasks/src/scripted_pick_demo.cpp",
        "src/robot_lab_perception/package.xml",
        "src/robot_lab_perception/setup.py",
        "src/robot_lab_perception/robot_lab_perception/object_pose_estimator.py",
        "src/robot_lab_perception/robot_lab_perception/detector_backends.py",
        "src/robot_lab_perception/robot_lab_perception/evaluate_perception.py",
        "README.md",
    ]

    missing = [path for path in expected_paths if not (ROOT / path).exists()]

    assert missing == []


def test_bringup_launch_is_self_contained_simulation():
    launch_file = ROOT / "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py"
    content = launch_file.read_text(encoding="utf-8")

    # Own xacro-processed robot description and Gazebo spawn chain.
    assert "lab_ur_gripper.urdf.xacro" in content
    assert "xacro.process_file" in content
    assert "gazebo.launch.py" in content
    assert '"gazebo_ros"' in content
    assert "spawn_entity.py" in content
    assert '"-topic"' in content
    assert "robot_state_publisher" in content
    assert "lab_minimal.sdf" in content
    assert "lab_ur_controllers.yaml" in content
    # Controller spawners include the gripper.
    assert '"joint_state_broadcaster"' in content
    assert '"joint_trajectory_controller"' in content
    assert '"gripper_position_controller"' in content
    # MoveIt is started directly (no ur_moveit.launch.py include).
    assert "ompl_planning_pipeline_config" in content
    assert "moveit_ros_move_group" in content
    assert "lab_ur_gripper.srdf.xacro" in content
    assert "ur_moveit.launch.py" not in content
    assert "ur_sim_control.launch.py" not in content


def test_bringup_uses_gazebo_classic_spawn_and_camera_topics():
    launch_file = ROOT / "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py"
    content = launch_file.read_text(encoding="utf-8")

    assert "gazebo_ros" in content
    assert "spawn_entity.py" in content
    assert "robot_description" in content
    assert "bench_camera_link" in content
    assert "ros_gz_bridge" not in content


def test_bringup_keeps_moveit_warehouse_path_available():
    launch_file = ROOT / "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py"
    content = launch_file.read_text(encoding="utf-8")

    assert 'LaunchConfiguration("warehouse_sqlite_path")' in content
    assert '"warehouse_sqlite_path"' in content
    assert '"warehouse_host": warehouse_sqlite_path' in content


def test_bringup_sets_wslg_d3d12_rendering_workaround():
    launch_file = ROOT / "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py"
    content = launch_file.read_text(encoding="utf-8")

    assert "SetEnvironmentVariable" in content
    assert '"QT_QPA_PLATFORM"' in content
    assert '"xcb"' in content
    assert '"QT_OPENGL"' in content
    assert "desktop" in content
    assert "software" in content
    assert '"GALLIUM_DRIVER"' in content
    assert "d3d12" in content
    assert '"MESA_D3D12_DEFAULT_ADAPTER_NAME"' in content
    assert "NVIDIA" in content
    assert '"QT_X11_NO_MITSHM"' in content
    assert '"1"' in content
    assert '"LIBGL_ALWAYS_SOFTWARE"' in content
    assert '"XDG_RUNTIME_DIR"' in content


def test_bringup_limits_gui_render_loops_by_default():
    launch_file = ROOT / "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py"
    content = launch_file.read_text(encoding="utf-8")

    assert '"sync_to_vblank"' in content
    assert 'default_value="true"' in content
    assert '"vblank_mode"' in content
    assert '"__GL_SYNC_TO_VBLANK"' in content
    assert 'SetEnvironmentVariable("vblank_mode", "0")' not in content
    assert 'SetEnvironmentVariable("__GL_SYNC_TO_VBLANK", "0")' not in content


def test_bringup_prevents_wslg_rviz_gazebo_gui_contention_by_default():
    launch_file = ROOT / "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py"
    content = launch_file.read_text(encoding="utf-8")

    assert '"allow_dual_gui"' in content
    assert "effective_gazebo_gui" in content
    assert "WSL_DISTRO_NAME" in content
    assert '"gui": effective_gazebo_gui' in content
    assert "Gazebo GUI disabled because RViz is also enabled under WSLg" in content


def test_bringup_recenters_rviz_window_for_wslg_multi_monitor():
    launch_file = ROOT / "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py"
    launch_content = launch_file.read_text(encoding="utf-8")
    cmake_content = (ROOT / "src/robot_lab_bringup/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    script_content = (
        ROOT / "src/robot_lab_bringup/scripts/recenter_rviz_window.py"
    ).read_text(encoding="utf-8")

    assert "ExecuteProcess" in launch_content
    assert "recenter_rviz_window.py" in launch_content
    assert "rviz_recenter" in launch_content
    assert "condition=IfCondition(launch_rviz)" in launch_content
    assert "scripts/recenter_rviz_window.py" in cmake_content
    assert "DESTINATION share/${PROJECT_NAME}/scripts" in cmake_content
    assert "xrandr" in script_content
    assert "XQueryTree" in script_content
    assert "XMoveWindow" in script_content
    assert "- RViz" in script_content


def test_project_rviz_config_starts_on_visible_primary_screen():
    rviz_config = ROOT / "src/robot_lab_bringup/config/robot_lab_moveit.rviz"
    content = rviz_config.read_text(encoding="utf-8")

    assert "moveit_rviz_plugin/MotionPlanning" in content
    assert "Window Geometry:" in content
    assert "Width: 1600" in content
    assert "Height: 1000" in content
    assert "X: 80" in content
    assert "Y: 80" in content


def test_robot_urdf_combines_ur_pedestal_gripper_and_grasp_plugins():
    urdf = (
        ROOT / "src/robot_lab_description/urdf/lab_ur_gripper.urdf.xacro"
    ).read_text(encoding="utf-8")

    assert "ur_macro.xacro" in urdf
    assert "parallel_gripper_macro.xacro" in urdf
    assert "pedestal_link" in urdf
    assert 'parent="pedestal_link"' in urdf
    assert "libgazebo_ros2_control.so" in urdf
    assert "gazebo_ros2_control/GazeboSystem" in urdf
    assert "gz::sim::systems::DetachableJoint" not in urdf
    assert "gripper_left_finger_joint" in urdf
    assert "gripper_right_finger_joint" in urdf


def test_srdf_extends_stock_semantics_with_gripper():
    srdf = (
        ROOT / "src/robot_lab_description/srdf/lab_ur_gripper.srdf.xacro"
    ).read_text(encoding="utf-8")

    assert "ur_macro.srdf.xacro" in srdf
    assert '<xacro:ur_srdf name="$(arg name)" prefix="$(arg prefix)"/>' in srdf
    assert '<group name="gripper">' in srdf
    assert 'link1="pedestal_link" link2="base_link"' in srdf
    assert 'link1="gripper_base" link2="wrist_3_link"' in srdf


def test_controllers_config_includes_arm_and_gripper():
    controllers = (
        ROOT / "src/robot_lab_bringup/config/lab_ur_controllers.yaml"
    ).read_text(encoding="utf-8")

    assert "scaled_joint_trajectory_controller" in controllers
    assert "ur_controllers/ScaledJointTrajectoryController" in controllers
    assert 'speed_scaling_interface_name: ""' in controllers
    assert "gripper_position_controller" in controllers
    assert "position_controllers/JointGroupPositionController" in controllers
    assert "gripper_left_finger_joint" in controllers
    assert "gripper_right_finger_joint" in controllers


def test_world_contains_manipulable_lab_objects():
    world = (
        ROOT / "src/robot_lab_description/worlds/lab_minimal.sdf"
    ).read_text(encoding="utf-8")

    assert "lab_workbench" in world
    assert "sample_block" in world
    assert "sample_vial_red" in world
    assert "sample_tray" in world
    assert "target_pad" in world
    assert "tool_caddy" in world


def test_world_has_overhead_depth_camera_with_gazebo_ros_plugin():
    world = (
        ROOT / "src/robot_lab_description/worlds/lab_minimal.sdf"
    ).read_text(encoding="utf-8")

    assert "libgazebo_ros_camera.so" in world
    assert 'type="depth"' in world
    assert "bench_camera" in world
    assert "frame_name>bench_camera_optical_frame" in world


def test_bringup_bridges_camera_topics_and_starts_perception():
    launch_file = ROOT / "src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py"
    content = launch_file.read_text(encoding="utf-8")

    assert "robot_lab_perception" in content
    assert "object_pose_estimator" in content
    assert '"launch_perception"' in content
    assert "bench_camera_link" in content


def test_perception_package_provides_backends_and_evaluation():
    estimator = (
        ROOT
        / "src/robot_lab_perception/robot_lab_perception/object_pose_estimator.py"
    ).read_text(encoding="utf-8")
    backends = (
        ROOT
        / "src/robot_lab_perception/robot_lab_perception/detector_backends.py"
    ).read_text(encoding="utf-8")
    evaluator = (
        ROOT
        / "src/robot_lab_perception/robot_lab_perception/evaluate_perception.py"
    ).read_text(encoding="utf-8")

    assert "/perception/" in estimator
    assert "_resolve_convention" in estimator
    assert "sample_block" in backends
    assert "sample_vial_red" in backends
    assert "OnnxDetectorBackend" in backends
    assert "set_pose" in evaluator
    assert "err_xy_mm" in evaluator


def test_task_layer_provides_orchestration_chain():
    orchestrator = (
        ROOT / "src/robot_lab_tasks/scripts/task_orchestrator.py"
    ).read_text(encoding="utf-8")
    pick_place = (
        ROOT / "src/robot_lab_tasks/src/pick_place_node.cpp"
    ).read_text(encoding="utf-8")
    monitor = (
        ROOT / "src/robot_lab_tasks/scripts/obstacle_monitor.py"
    ).read_text(encoding="utf-8")
    evaluator = (
        ROOT / "src/robot_lab_tasks/scripts/evaluate_pick_place.py"
    ).read_text(encoding="utf-8")

    # Text instruction -> decomposition -> queue -> dispatch.
    assert "/task/instruction" in orchestrator
    assert "/task/command" in orchestrator
    assert "RuleBasedPlanner" in orchestrator
    assert "LlmPlanner" in orchestrator
    # Perception-driven execution with retry-replan.
    assert "/perception/" in pick_place
    assert "/compute_ik" in pick_place
    assert "with_retry" in pick_place
    assert "kFamilySeed" in pick_place
    # Dynamic obstacle -> planning scene with latency evidence.
    assert "/planning_scene" in monitor
    assert "obstacle_latency" in monitor
    assert "moving_obstacle" in monitor
    assert "sweep_obstacle" in evaluator


def test_world_has_moving_obstacle_with_pose_publisher():
    world = (
        ROOT / "src/robot_lab_description/worlds/lab_minimal.sdf"
    ).read_text(encoding="utf-8")

    assert "moving_obstacle" in world
    assert "PosePublisher" not in world


def test_mecanum_variant_provides_omnidirectional_base():
    base = (
        ROOT / "src/robot_lab_description/urdf/inc/mecanum_base_macro.xacro"
    ).read_text(encoding="utf-8")
    robot = (
        ROOT / "src/robot_lab_description/urdf/lab_ur_mecanum.urdf.xacro"
    ).read_text(encoding="utf-8")
    launch = (
        ROOT / "src/robot_lab_bringup/launch/lab_ur_mecanum_gz.launch.py"
    ).read_text(encoding="utf-8")
    world = (
        ROOT / "src/robot_lab_description/worlds/lab_minimal.sdf"
    ).read_text(encoding="utf-8")

    # Four wheels with X-pattern roller friction.
    for wheel in ("front_left", "front_right", "rear_left", "rear_right"):
        assert f"{wheel}_wheel" in base
    assert base.count("<fdir1") == 4
    # MecanumDrive system wired to /cmd_vel + /odom.
    assert "gz::sim::systems::MecanumDrive" in robot
    assert "<topic>/cmd_vel</topic>" in robot
    assert "<odom_topic>/odom</odom_topic>" in robot
    assert "base_footprint" in robot
    # Launch bridges drive topics and reuses the arm/gripper stack.
    assert "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" in launch
    assert "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" in launch
    assert "lab_ur_mecanum.urdf.xacro" in launch
    # Cross-station target beyond the fixed arm's reach.
    assert "station_b" in world


def test_scripted_pick_demo_closes_the_grasp_loop():
    task_file = ROOT / "src/robot_lab_tasks/src/scripted_pick_demo.cpp"
    content = task_file.read_text(encoding="utf-8")

    assert '"ur_manipulator"' in content
    assert "MoveGroupInterface" in content
    assert "/gripper_position_controller/commands" in content
    assert "/gripper/attach/sample_block" in content
    assert "/gripper/detach/sample_block" in content
    assert "computeCartesianPath" in content
    assert "lab_table" in content


def test_preset_joint_demo_remains_as_smoke_test():
    task_file = ROOT / "src/robot_lab_tasks/src/preset_joint_demo.cpp"
    content = task_file.read_text(encoding="utf-8")

    assert '"ur_manipulator"' in content
    assert "MoveGroupInterface" in content
    assert "setJointValueTarget(target.joints)" in content


def test_readme_documents_demo_commands():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")

    assert "ros2 launch robot_lab_bringup lab_ur_moveit_gz.launch.py" in readme
    assert "ros2 run robot_lab_tasks scripted_pick_demo" in readme
    assert "ros2 run robot_lab_tasks preset_joint_demo" in readme
    assert "ros2 control list_controllers" in readme
