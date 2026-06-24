"""Mobile variant of the lab demo: UR5e + gripper on a mecanum base.

Same stack as lab_ur_moveit_gz.launch.py (Gazebo, MoveIt, perception, task
layer) but the robot rides a four-wheel mecanum base driven over /cmd_vel
(gz-sim MecanumDrive system, odometry on /odom). With the base parked at
the world origin the arm reproduces the fixed demo's grasp geometry;
driving to station_b (x=2.2) demonstrates the cross-station workflow.

Teleop example:
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.3, y: 0.1}, angular: {z: 0.0}}"
"""

import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context).lower() in (
        "true",
        "1",
    )
    launch_rviz = LaunchConfiguration("launch_rviz")
    show_lab_scene = LaunchConfiguration("show_lab_scene")
    launch_perception = LaunchConfiguration("launch_perception")
    launch_tasks = LaunchConfiguration("launch_tasks")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path").perform(
        context
    )

    description_pkg = get_package_share_directory("robot_lab_description")
    bringup_pkg = get_package_share_directory("robot_lab_bringup")

    world_file = os.path.join(description_pkg, "worlds", "lab_minimal.sdf")
    urdf_xacro = os.path.join(description_pkg, "urdf", "lab_ur_mecanum.urdf.xacro")
    srdf_xacro = os.path.join(description_pkg, "srdf", "lab_ur_gripper.srdf.xacro")
    controllers_yaml = os.path.join(bringup_pkg, "config", "lab_ur_controllers.yaml")
    rviz_config = os.path.join(bringup_pkg, "config", "robot_lab_moveit.rviz")
    lab_scene_markers_script = os.path.join(
        bringup_pkg, "scripts", "static_lab_scene_markers.py"
    )

    robot_description_content = xacro.process_file(
        urdf_xacro,
        mappings={
            "name": "ur",
            "ur_type": ur_type,
            "tf_prefix": "",
            "safety_limits": "true",
            "simulation_controllers": controllers_yaml,
        },
    ).toxml()
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path(srdf_xacro), {"name": "ur"})
        .to_moveit_configs()
    )
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    gz_args = f" -r -v 4 {world_file}" if gazebo_gui else f" -s -r -v 4 {world_file}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    gz_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
            "-z",
            "0.03",
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/gripper/attach/sample_block@std_msgs/msg/Empty]gz.msgs.Empty",
            "/gripper/detach/sample_block@std_msgs/msg/Empty]gz.msgs.Empty",
            "/gripper/attach/sample_vial_red@std_msgs/msg/Empty]gz.msgs.Empty",
            "/gripper/detach/sample_vial_red@std_msgs/msg/Empty]gz.msgs.Empty",
            "/bench_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/bench_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/model/moving_obstacle/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        ],
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bench_camera_tf",
        output="screen",
        arguments=[
            "--x", "0.62", "--y", "0.0", "--z", "2.0",
            "--roll", "0.0", "--pitch", "1.5708", "--yaw", "0.0",
            "--frame-id", "world", "--child-frame-id", "bench_camera_link",
        ],
    )
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bench_camera_optical_tf",
        output="screen",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "-1.57079632679", "--pitch", "0.0", "--yaw", "-1.57079632679",
            "--frame-id", "bench_camera_link",
            "--child-frame-id", "bench_camera_optical_frame",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    def controller_spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                controller_name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "120",
            ],
        )

    spawners = [
        controller_spawner("joint_state_broadcaster"),
        controller_spawner("scaled_joint_trajectory_controller"),
        controller_spawner("gripper_position_controller"),
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            moveit_config.to_dict(),
            warehouse_ros_config,
            {"use_sim_time": True, "publish_robot_description_semantic": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )
    lab_scene_markers = ExecuteProcess(
        cmd=[lab_scene_markers_script],
        output="screen",
        condition=IfCondition(show_lab_scene),
    )

    perception_node = Node(
        package="robot_lab_perception",
        executable="object_pose_estimator",
        name="object_pose_estimator",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_perception),
    )

    task_orchestrator = Node(
        package="robot_lab_tasks",
        executable="task_orchestrator.py",
        name="task_orchestrator",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_tasks),
    )
    obstacle_monitor = Node(
        package="robot_lab_tasks",
        executable="obstacle_monitor.py",
        name="obstacle_monitor",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"latency_csv": "results/obstacle_latency.csv"},
        ],
        condition=IfCondition(launch_tasks),
    )
    pick_place = TimerAction(
        period=18.0,
        actions=[
            Node(
                package="robot_lab_tasks",
                executable="pick_place_node",
                name="pick_place_node",
                output="screen",
                condition=IfCondition(launch_tasks),
            )
        ],
    )

    return [
        gazebo,
        gz_spawn_robot,
        gz_bridge,
        camera_tf,
        camera_optical_tf,
        perception_node,
        lab_scene_markers,
        task_orchestrator,
        obstacle_monitor,
        pick_place,
        robot_state_publisher,
        *spawners,
        move_group,
        rviz_node,
    ]


def generate_launch_description():
    runtime_dir = f"/tmp/runtime-{os.environ.get('USER', 'robot_lab')}"
    Path(runtime_dir).mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(runtime_dir, 0o700)

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("show_lab_scene", default_value="false"),
            DeclareLaunchArgument("launch_perception", default_value="true"),
            DeclareLaunchArgument("launch_tasks", default_value="true"),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            ),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            SetEnvironmentVariable("QT_OPENGL", "desktop"),
            SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12"),
            SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA"),
            SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
            SetEnvironmentVariable("XDG_RUNTIME_DIR", runtime_dir),
            OpaqueFunction(function=launch_setup),
        ]
    )
