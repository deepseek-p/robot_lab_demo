import os
import re
from pathlib import Path

import xacro
import yaml
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def load_yaml_file(path):
    with open(path, "r", encoding="utf-8") as yaml_file:
        return yaml.safe_load(yaml_file)


def load_ros_parameters(path):
    data = load_yaml_file(path)
    return data.get("/**", {}).get("ros__parameters", data)


def strip_xml_comments(xml_text):
    xml_text = re.sub(r"<\?xml[^>]*\?>", "", xml_text)
    xml_text = re.sub(r"<!--.*?-->", "", xml_text, flags=re.DOTALL)
    return xml_text.strip()


def launch_setup(context, *args, **kwargs):
    """Start the lab demo on ROS 2 Humble + Gazebo Classic."""
    ur_type = LaunchConfiguration("ur_type").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")
    show_lab_scene = LaunchConfiguration("show_lab_scene")
    publish_scene_objects = LaunchConfiguration("publish_scene_objects")
    launch_perception = LaunchConfiguration("launch_perception")
    launch_tasks = LaunchConfiguration("launch_tasks")
    auto_start_task = LaunchConfiguration("auto_start_task")
    rviz_profile = LaunchConfiguration("rviz_profile").perform(context).lower()
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path").perform(
        context
    )

    description_pkg = get_package_share_directory("robot_lab_description")
    bringup_pkg = get_package_share_directory("robot_lab_bringup")
    ur_moveit_pkg = get_package_share_directory("ur_moveit_config")

    world_file = os.path.join(description_pkg, "worlds", "lab_minimal.sdf")
    urdf_xacro = os.path.join(description_pkg, "urdf", "lab_ur_gripper.urdf.xacro")
    srdf_xacro = os.path.join(description_pkg, "srdf", "lab_ur_gripper.srdf.xacro")
    controllers_yaml = os.path.join(bringup_pkg, "config", "lab_ur_controllers.yaml")
    rviz_config_name = (
        "robot_lab_moveit.rviz"
        if rviz_profile in ("moveit", "full")
        else "robot_lab_stable.rviz"
    )
    rviz_config = os.path.join(bringup_pkg, "config", rviz_config_name)
    rviz_recenter_script = os.path.join(
        bringup_pkg, "scripts", "recenter_rviz_window.py"
    )
    lab_scene_markers_script = os.path.join(
        bringup_pkg, "scripts", "static_lab_scene_markers.py"
    )

    robot_description_content = strip_xml_comments(
        xacro.process_file(
            urdf_xacro,
            mappings={
                "name": "ur",
                "ur_type": ur_type,
                "tf_prefix": "",
                "safety_limits": "true",
                "simulation_controllers": controllers_yaml,
            },
        ).toxml()
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic = {
        "robot_description_semantic": xacro.process_file(
            srdf_xacro, mappings={"name": "ur", "prefix": ""}
        ).toxml()
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_ros_parameters(
            os.path.join(ur_moveit_pkg, "config", "kinematics.yaml")
        )["robot_description_kinematics"]
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml_file(
            os.path.join(ur_moveit_pkg, "config", "joint_limits.yaml")
        )
    }
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(
        load_yaml_file(os.path.join(ur_moveit_pkg, "config", "ompl_planning.yaml"))
    )
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }
    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["joint_trajectory_controller"],
            "joint_trajectory_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": ARM_JOINTS,
            },
        },
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file,
            "gui": LaunchConfiguration("gazebo_gui"),
            "verbose": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    "ur",
                    "-z",
                    "0.0",
                    "-timeout",
                    "120",
                    "-package_to_model",
                ],
            )
        ],
    )

    def controller_spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "120",
            ],
        )

    spawners = TimerAction(
        period=7.0,
        actions=[
            controller_spawner("joint_state_broadcaster"),
            controller_spawner("joint_trajectory_controller"),
            controller_spawner("gripper_position_controller"),
        ],
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bench_camera_tf",
        output="screen",
        arguments=[
            "--x",
            "0.62",
            "--y",
            "0.0",
            "--z",
            "2.0",
            "--roll",
            "0.0",
            "--pitch",
            "1.5708",
            "--yaw",
            "0.0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "bench_camera_link",
        ],
    )
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bench_camera_optical_tf",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "-1.57079632679",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.57079632679",
            "--frame-id",
            "bench_camera_link",
            "--child-frame-id",
            "bench_camera_optical_frame",
        ],
    )

    perception_node = Node(
        package="robot_lab_perception",
        executable="object_pose_estimator",
        name="object_pose_estimator",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_perception),
    )
    planning_scene_objects = Node(
        package="robot_lab_tasks",
        executable="planning_scene_object_publisher.py",
        name="planning_scene_object_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(publish_scene_objects),
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
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    moveit_controllers,
                    {"use_sim_time": True},
                ],
                condition=IfCondition(launch_tasks),
            )
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            planning_scene_monitor_parameters,
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
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            warehouse_ros_config,
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )
    rviz_recenter = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[rviz_recenter_script],
                output="screen",
                condition=IfCondition(launch_rviz),
            )
        ],
    )
    lab_scene_markers = ExecuteProcess(
        cmd=[lab_scene_markers_script],
        output="screen",
        condition=IfCondition(show_lab_scene),
    )

    scripted_pick = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="robot_lab_tasks",
                executable="scripted_pick_demo",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    moveit_controllers,
                    {"use_sim_time": True},
                ],
                condition=IfCondition(auto_start_task),
            )
        ],
    )

    return [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        camera_tf,
        camera_optical_tf,
        spawners,
        move_group,
        perception_node,
        planning_scene_objects,
        lab_scene_markers,
        task_orchestrator,
        obstacle_monitor,
        pick_place,
        rviz_node,
        rviz_recenter,
        scripted_pick,
    ]


def generate_launch_description():
    runtime_dir = f"/tmp/runtime-{os.environ.get('USER', 'robot_lab')}"
    Path(runtime_dir).mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(runtime_dir, 0o700)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="UR robot type to simulate. The demo defaults to UR5e.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="false",
                description="Start Gazebo with GUI. Use false for headless smoke tests.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with the MoveIt planning interface.",
            ),
            DeclareLaunchArgument(
                "show_lab_scene",
                default_value="false",
                description="Show RViz markers for the lab table and objects. Keep false for robot-only study.",
            ),
            DeclareLaunchArgument(
                "publish_scene_objects",
                default_value="false",
                description="Publish LabScene/perception objects as MoveIt planning scene collision objects.",
            ),
            DeclareLaunchArgument(
                "launch_perception",
                default_value="false",
                description="Start the RGB-D object pose estimation pipeline.",
            ),
            DeclareLaunchArgument(
                "launch_tasks",
                default_value="false",
                description="Start the task orchestrator, obstacle monitor, and pick/place executor.",
            ),
            DeclareLaunchArgument(
                "rviz_profile",
                default_value="stable",
                description="RViz profile: stable uses RobotModel only; moveit loads the full MoveIt planning panel.",
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                description="Path where the MoveIt warehouse database should be stored.",
            ),
            DeclareLaunchArgument(
                "auto_start_task",
                default_value="false",
                description="Run the scripted pick-and-place demo automatically after startup.",
            ),
            DeclareLaunchArgument(
                "gui_scale",
                default_value="1.0",
                description="Scale factor for WSLg Qt/GTK GUI windows such as RViz and Gazebo.",
            ),
            DeclareLaunchArgument(
                "gui_font_dpi",
                default_value="144",
                description="Font DPI for Qt GUI windows such as RViz and Gazebo.",
            ),
            DeclareLaunchArgument(
                "cursor_size",
                default_value="48",
                description="X cursor size for Linux GUI windows launched by this file.",
            ),
            DeclareLaunchArgument(
                "render_mode",
                default_value="software",
                description="WSLg rendering mode: software is stable CPU rendering; gpu uses Mesa D3D12 acceleration.",
            ),
            DeclareLaunchArgument(
                "qt_gl_integration",
                default_value="xcb_egl",
                description="Qt XCB OpenGL integration: try xcb_egl first under WSLg; use xcb_glx if EGL flickers.",
            ),
            DeclareLaunchArgument(
                "window_backend",
                default_value="xcb",
                description="Qt platform backend: xcb works by default; wayland requires qtwayland5.",
            ),
            SetEnvironmentVariable("QT_QPA_PLATFORM", LaunchConfiguration("window_backend")),
            SetEnvironmentVariable(
                "QT_XCB_GL_INTEGRATION",
                LaunchConfiguration("qt_gl_integration"),
            ),
            SetEnvironmentVariable(
                "QT_OPENGL",
                PythonExpression(
                    [
                        "'desktop' if '",
                        LaunchConfiguration("render_mode"),
                        "' == 'gpu' else 'software'",
                    ]
                ),
            ),
            SetEnvironmentVariable("QT_ENABLE_HIGHDPI_SCALING", "1"),
            SetEnvironmentVariable("QT_AUTO_SCREEN_SCALE_FACTOR", "0"),
            SetEnvironmentVariable(
                "QT_SCALE_FACTOR", LaunchConfiguration("gui_scale")
            ),
            SetEnvironmentVariable("QT_SCALE_FACTOR_ROUNDING_POLICY", "PassThrough"),
            SetEnvironmentVariable("QT_FONT_DPI", LaunchConfiguration("gui_font_dpi")),
            SetEnvironmentVariable("GDK_SCALE", "1"),
            SetEnvironmentVariable("GDK_DPI_SCALE", LaunchConfiguration("gui_scale")),
            SetEnvironmentVariable("XCURSOR_THEME", "Adwaita"),
            SetEnvironmentVariable("XCURSOR_SIZE", LaunchConfiguration("cursor_size")),
            SetEnvironmentVariable("vblank_mode", "0"),
            SetEnvironmentVariable("__GL_SYNC_TO_VBLANK", "0"),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE",
                PythonExpression(
                    [
                        "'0' if '",
                        LaunchConfiguration("render_mode"),
                        "' == 'gpu' else '1'",
                    ]
                ),
            ),
            SetEnvironmentVariable(
                "GALLIUM_DRIVER",
                PythonExpression(
                    [
                        "'d3d12' if '",
                        LaunchConfiguration("render_mode"),
                        "' == 'gpu' else ''",
                    ]
                ),
            ),
            SetEnvironmentVariable(
                "MESA_D3D12_DEFAULT_ADAPTER_NAME",
                PythonExpression(
                    [
                        "'NVIDIA' if '",
                        LaunchConfiguration("render_mode"),
                        "' == 'gpu' else ''",
                    ]
                ),
            ),
            SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
            SetEnvironmentVariable("XDG_RUNTIME_DIR", runtime_dir),
            OpaqueFunction(function=launch_setup),
        ]
    )
