from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_world_exposes_gazebo_classic_state_services():
    world = read("src/robot_lab_description/worlds/lab_minimal.sdf")

    assert 'filename="libgazebo_ros_state.so"' in world
    assert "<namespace>/gazebo</namespace>" in world
    assert "libgazebo_ros_camera.so" in world
    assert "gz::sim::systems::PosePublisher" not in world


def test_evaluators_use_gazebo_msgs_instead_of_gz_cli():
    perception_eval = read(
        "src/robot_lab_perception/robot_lab_perception/evaluate_perception.py"
    )
    pick_eval = read("src/robot_lab_tasks/scripts/evaluate_pick_place.py")

    for content in (perception_eval, pick_eval):
        assert "robot_lab_sim_tools.gazebo_state" in content
        assert "subprocess" not in content
        assert '"gz"' not in content
        assert "/world/robot_lab_minimal/set_pose" not in content


def test_obstacle_monitor_uses_gazebo_classic_model_states():
    monitor = read("src/robot_lab_tasks/scripts/obstacle_monitor.py")

    assert "from gazebo_msgs.msg import ModelStates" in monitor
    assert '"/gazebo/model_states"' in monitor
    assert '"/model/moving_obstacle/pose"' not in monitor
    assert "PosePublisher" not in monitor


def test_soft_gripper_attach_bridge_package_and_launch_exist():
    setup_py = read("src/robot_lab_sim_tools/setup.py")
    setup_cfg = read("src/robot_lab_sim_tools/setup.cfg")
    launch = read("src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py")
    package_xml = read("src/robot_lab_bringup/package.xml")

    assert (
        "soft_gripper_attach = robot_lab_sim_tools.soft_gripper_attach:main"
        in setup_py
    )
    assert "script_dir=$base/lib/robot_lab_sim_tools" in setup_cfg
    assert "install_scripts=$base/lib/robot_lab_sim_tools" in setup_cfg
    assert 'package="robot_lab_sim_tools"' in launch
    assert 'executable="soft_gripper_attach"' in launch
    assert "<exec_depend>robot_lab_sim_tools</exec_depend>" in package_xml


def test_classic_attach_bridge_consumes_existing_gripper_topics():
    bridge = read("src/robot_lab_sim_tools/robot_lab_sim_tools/soft_gripper_attach.py")

    assert "/gripper/attach/sample_block" in bridge
    assert "/gripper/detach/sample_block" in bridge
    assert "/gripper/attach/sample_vial_red" in bridge
    assert "/gripper/detach/sample_vial_red" in bridge
    assert "SetEntityState" in bridge
    assert "/gazebo/model_states" in bridge
    assert "self._subscriptions =" not in bridge
    assert "self._owned_subscriptions" in bridge


def test_mecanum_variant_is_marked_experimental_when_gz_sim_deps_are_absent():
    launch = read("src/robot_lab_bringup/launch/lab_ur_mecanum_gz.launch.py")
    readme = read("README.md")

    assert "require_gazebo_sim_package" in launch
    assert "ros_gz_sim" in launch
    assert "ros_gz_bridge" in launch
    assert "实验" in readme or "experimental" in readme.lower()
