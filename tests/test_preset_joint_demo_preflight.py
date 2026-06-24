import subprocess

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_preset_joint_demo_explains_missing_bringup_when_run_alone():
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ROS_DOMAIN_ID=217 timeout 20 ros2 run robot_lab_tasks preset_joint_demo"
    )

    result = subprocess.run(
        ["bash", "-lc", command],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=25,
        check=False,
    )

    assert result.returncode == 2
    assert "Start the demo bringup first" in result.stdout
    assert "ros2 launch robot_lab_bringup lab_ur_moveit_gz.launch.py" in result.stdout
