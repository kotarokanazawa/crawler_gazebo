#!/usr/bin/env python3
import os
import subprocess
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in {"1", "true", "yes", "on"}


def _read_yaml(path):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}
    except FileNotFoundError:
        return {}


def _setting(settings, key, default):
    node = settings
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _xacro_to_string(model, xacro_args):
    cmd = ["xacro", model] + [f"{name}:={value}" for name, value in xacro_args.items()]
    return subprocess.check_output(cmd, text=True)


def _strip_xml_declaration(xml):
    stripped = xml.lstrip()
    if stripped.startswith("<?xml"):
        return stripped.split("?>", 1)[1].lstrip()
    return xml


def _write_spawn_urdf(xml, name):
    path = Path(tempfile.gettempdir()) / f"crawler_gazebo_{name}.urdf"
    path.write_text(_strip_xml_declaration(xml), encoding="utf-8")
    return path


def _with_ros2_control_yaml(xml, control_yaml):
    return xml.replace("__CRAWLER_ROS2_CONTROL_YAML__", str(control_yaml))


def _default_spawn_z(crawler_gazebo_share, robot_size):
    robot_yaml = crawler_gazebo_share / "config" / "robot" / f"{robot_size}.yaml"
    config = _read_yaml(robot_yaml).get("robot_config", {})
    wheel_radius = float(_setting(config, "geometry.wheel_radius", 0.082))
    belt_thickness = float(_setting(config, "continuous_track.belt_thickness", 0.02))
    grouser_height = float(_setting(config, "continuous_track.grouser_height", 0.01))
    spawn_clearance = 0.05
    return f"{wheel_radius + belt_thickness + grouser_height + spawn_clearance:.3f}"


def _optional_package_lib(package_name):
    try:
        return str(Path(get_package_prefix(package_name)) / "lib")
    except PackageNotFoundError:
        print(f"[crawler_gazebo] Optional package not found, skipping plugin path: {package_name}")
        return ""


def _crawler_gazebo_package_root(package_share):
    env_source = os.environ.get("CRAWLER_GAZEBO_SOURCE_DIR", "")
    if env_source:
        source = Path(env_source).expanduser()
        if (source / "package.xml").exists():
            return source

    for parent in [package_share, *package_share.parents]:
        if parent.name != "install":
            continue
        workspace = parent.parent
        for package_xml in (workspace / "src").glob("**/crawler_gazebo/package.xml"):
            return package_xml.parent
    return package_share


def _resolve_package_file(value, package_share, subdir=""):
    package_root = _crawler_gazebo_package_root(package_share)
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    if str(value).startswith("package://crawler_gazebo/"):
        return package_root / str(value)[len("package://crawler_gazebo/"):]
    return package_root / subdir / path


def _launch_setup(context, *args, **kwargs):
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    crawler_description_share = Path(get_package_share_directory("crawler_description"))
    crawler_ros_control_share = Path(get_package_share_directory("crawler_ros_control"))
    continuous_track_lib = _optional_package_lib("gazebo_continuous_track_ros2_gazebo11")

    robot_size = LaunchConfiguration("robot_size").perform(context)
    robot_urdf_arg = LaunchConfiguration("robot_urdf").perform(context)
    robot_urdf = Path(robot_urdf_arg) if robot_urdf_arg else crawler_gazebo_share / "urdf" / f"{robot_size}_crawler.urdf"
    model = LaunchConfiguration("model").perform(context)
    worldfile = _resolve_package_file(
        LaunchConfiguration("worldfile").perform(context), crawler_gazebo_share, "world")
    simsetting_yaml = _resolve_package_file(
        LaunchConfiguration("simsetting_yaml").perform(context), crawler_gazebo_share, "config")
    arena_yaml = _resolve_package_file(
        LaunchConfiguration("arena_yaml").perform(context), crawler_gazebo_share, "config/gazebo_environment")

    settings = _read_yaml(simsetting_yaml)
    spawn_robot = _as_bool(_setting(settings, "crawler_gazebo.simulator.spawn_robot", True))
    spawn_arena_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.spawn_arena", True))
    start_control_nodes_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.start_control_nodes", True))
    start_controllers_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.start_controllers", True))
    start_gui_tools_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.start_gui_tools", True))
    start_flipper_joint_gui_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.start_flipper_joint_gui", True))
    start_cloudmap_publisher_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.start_cloudmap_publisher", True))

    norobot = _as_bool(LaunchConfiguration("norobot").perform(context))
    spawn_arena = _as_bool(LaunchConfiguration("spawn_arena").perform(context)) and spawn_arena_setting
    start_control_nodes = _as_bool(LaunchConfiguration("start_control_nodes").perform(context)) and start_control_nodes_setting
    start_controllers = _as_bool(LaunchConfiguration("start_controllers").perform(context)) and start_controllers_setting
    start_gui_tools = _as_bool(LaunchConfiguration("start_gui_tools").perform(context)) and start_gui_tools_setting
    start_flipper_joint_gui = (
        _as_bool(LaunchConfiguration("start_flipper_joint_gui").perform(context))
        and start_flipper_joint_gui_setting
    )
    start_cloudmap_publisher = (
        _as_bool(LaunchConfiguration("start_cloudmap_publisher").perform(context))
        and start_cloudmap_publisher_setting
    )
    use_generated_robot_urdf = _as_bool(LaunchConfiguration("use_generated_robot_urdf").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    if spawn_z == "auto":
        spawn_z = _default_spawn_z(crawler_gazebo_share, robot_size)
    control_yaml = LaunchConfiguration("control_yaml").perform(context)
    if not control_yaml:
        control_yaml = str(crawler_ros_control_share / "config" / "gazebo" / "ros2_controllers.yaml")
    control_params_path = crawler_ros_control_share / "config" / "gazebo" / "control.yaml"
    cloud_resolution = float(_setting(settings, "gazebo_to_octomap_publisher.resolution", 0.025))

    gazebo_plugin_paths = [
        path for path in [
            continuous_track_lib,
            str(crawler_gazebo_share.parent.parent / "lib"),
            os.environ.get("GAZEBO_PLUGIN_PATH", ""),
        ]
        if path
    ]

    actions = [
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            f"{crawler_gazebo_share / 'gazebo_model' / 'model'}:{os.environ.get('GAZEBO_MODEL_PATH', '')}",
        ),
        SetEnvironmentVariable(
            "GAZEBO_PLUGIN_PATH",
            ":".join(gazebo_plugin_paths),
        ),
    ]

    gzserver_cmd = [
        "gzserver",
        "-s", "libgazebo_ros_init.so",
        "-s", "libgazebo_ros_factory.so",
        "-s", "libgazebo_ros_state.so",
        str(worldfile),
    ]
    if _as_bool(LaunchConfiguration("paused").perform(context)):
        gzserver_cmd.append("--pause")
    actions.append(ExecuteProcess(cmd=gzserver_cmd, output="screen"))

    if _as_bool(LaunchConfiguration("gui").perform(context)):
        actions.append(ExecuteProcess(cmd=["gzclient"], output="screen"))

    if spawn_robot and not norobot:
        if use_generated_robot_urdf:
            robot_description = _with_ros2_control_yaml(
                _strip_xml_declaration(robot_urdf.read_text(encoding="utf-8")),
                control_yaml,
            )
            spawn_urdf = _write_spawn_urdf(robot_description, robot_urdf.stem)
            spawn_args = [
                "-entity", "crawler",
                "-file", str(spawn_urdf),
                "-z", spawn_z,
                "-timeout", "120",
            ]
        else:
            robot_description = _with_ros2_control_yaml(
                _xacro_to_string(
                    model,
                    {
                        "enable_body_tracks": LaunchConfiguration("enable_body_tracks").perform(context),
                        "enable_flipper_tracks": LaunchConfiguration("enable_flipper_tracks").perform(context),
                        "enable_gazebo_ros_control": LaunchConfiguration("enable_gazebo_ros_control").perform(context),
                        "enable_imu": LaunchConfiguration("enable_imu").perform(context),
                        "enable_position_transmissions": LaunchConfiguration("enable_position_transmissions").perform(context),
                        "enable_velocity_transmissions": LaunchConfiguration("enable_velocity_transmissions").perform(context),
                        "enable_body_velocity_transmissions": LaunchConfiguration("enable_body_velocity_transmissions").perform(context),
                        "enable_flipper_velocity_transmissions": LaunchConfiguration("enable_flipper_velocity_transmissions").perform(context),
                    },
                ),
                control_yaml,
            )
            spawn_args = [
                "-entity", "crawler",
                "-topic", "robot_description",
                "-z", spawn_z,
                "-timeout", "120",
            ]

        actions.extend([
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="log",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="urdf_spawner",
                output="screen",
                arguments=spawn_args,
                additional_env={"PATH": f"/usr/bin:{os.environ.get('PATH', '')}"},
            ),
        ])

        if start_control_nodes:
            actions.extend([
                Node(
                    package="crawler_ros_control",
                    executable="twist_gazebo",
                    name="gazebo_track_move",
                    output="log",
                    parameters=[str(control_params_path)],
                ),
                Node(
                    package="crawler_ros_control",
                    executable="flipper_gazebo",
                    name="gazebo_flipper",
                    output="log",
                    parameters=[str(control_params_path)],
                ),
                Node(
                    package="crawler_ros_control",
                    executable="gazebo_pose",
                    name="gazebo_pose_publisher",
                    output="log",
                    parameters=[str(control_params_path)],
                ),
            ])

        if start_controllers:
            actions.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    name="crawler_controller_spawner",
                    output="screen",
                    arguments=[
                        "joint_state_broadcaster",
                        "sprocket_velocity_controller_sprocket_axle_left",
                        "sprocket_velocity_controller_sprocket_axle_right",
                        "sprocket_velocity_controller_flipper_sprocket_axle_left_front",
                        "sprocket_velocity_controller_flipper_sprocket_axle_left_rear",
                        "sprocket_velocity_controller_flipper_sprocket_axle_right_front",
                        "sprocket_velocity_controller_flipper_sprocket_axle_right_rear",
                        "flipper_controller_LF",
                        "flipper_controller_LB",
                        "flipper_controller_RF",
                        "flipper_controller_RB",
                        "--controller-manager",
                        "/controller_manager",
                        "--param-file",
                        control_yaml,
                        "--controller-manager-timeout",
                        "120",
                        "--switch-timeout",
                        "120",
                        "--activate-as-group",
                    ],
                )
            )

        if start_gui_tools:
            actions.append(
                Node(
                    package="rqt_robot_steering",
                    executable="rqt_robot_steering",
                    name="rqt_robot_steering",
                    output="log",
                    remappings=[("/cmd_vel", "/target/cmd_vel")],
                )
            )

        if start_flipper_joint_gui:
            flipper_model = crawler_description_share / "xacro" / "flipper_joint_gui.xacro"
            try:
                flipper_description = _xacro_to_string(str(flipper_model), {})
                actions.append(
                    Node(
                        package="joint_state_publisher_gui",
                        executable="joint_state_publisher_gui",
                        name="flipper_joint_state_publisher_gui",
                        output="screen",
                        parameters=[{"robot_description": flipper_description, "use_sim_time": use_sim_time}],
                        remappings=[("joint_states", "/target/joint_states")],
                    )
                )
            except Exception as exc:
                print(f"[crawler_gazebo] flipper_joint_gui xacro skipped: {exc}")

    if spawn_arena:
        actions.append(
            Node(
                package="crawler_gazebo",
                executable="spawn_robocup_arena.py",
                name="spawn_robocup_arena",
                output="screen",
                parameters=[{
                    "arena_yaml": str(arena_yaml),
                    "use_wall_arg": _as_bool(LaunchConfiguration("use_wall_arg").perform(context)),
                    "wall_sdf": LaunchConfiguration("wall_sdf").perform(context),
                }],
                additional_env={"PATH": f"/usr/bin:{os.environ.get('PATH', '')}"},
            )
        )

    if start_cloudmap_publisher:
        actions.extend([
            Node(
                package="crawler_gazebo",
                executable="gazebo_to_octomap_publisher",
                name="gazebo_to_octomap_publisher",
                output="screen",
                parameters=[
                    {
                        "arena_yaml": str(arena_yaml),
                        "model_root": str(crawler_gazebo_share / "gazebo_model" / "model"),
                        "use_gazebo_services": True,
                        "use_sim_time": use_sim_time,
                        "resolution": cloud_resolution,
                        "plane_step": cloud_resolution,
                    },
                ],
            ),
            Node(
                package="crawler_gazebo",
                executable="voxel_overhang_removal",
                name="voxel_overhang_removal",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/octomap_pointcloud",
                        "output_topic": "/octomap_pointcloud/filtering",
                        "voxel_size": 0.05,
                        "height_mode": "max",
                        "durability": "transient_local",
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ])

    return actions


def generate_launch_description():
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    crawler_description_share = Path(get_package_share_directory("crawler_description"))

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=str(crawler_description_share / "xacro" / "crawler_track.xacro")),
        DeclareLaunchArgument("robot_size", default_value="default"),
        DeclareLaunchArgument("robot_urdf", default_value=""),
        DeclareLaunchArgument("use_generated_robot_urdf", default_value="true"),
        DeclareLaunchArgument("worldfile", default_value="base_fields.world"),
        DeclareLaunchArgument("simsetting_yaml", default_value="simsetting.yaml"),
        DeclareLaunchArgument("arena_yaml", default_value="benchmark.yaml"),
        DeclareLaunchArgument("spawn_arena", default_value="true"),
        DeclareLaunchArgument("use_wall_arg", default_value="false"),
        DeclareLaunchArgument("wall_sdf", default_value=""),
        DeclareLaunchArgument("norobot", default_value="false"),
        DeclareLaunchArgument("enable_body_tracks", default_value="true"),
        DeclareLaunchArgument("enable_flipper_tracks", default_value="true"),
        DeclareLaunchArgument("enable_gazebo_ros_control", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("enable_position_transmissions", default_value="true"),
        DeclareLaunchArgument("enable_velocity_transmissions", default_value="true"),
        DeclareLaunchArgument("enable_body_velocity_transmissions", default_value="true"),
        DeclareLaunchArgument("enable_flipper_velocity_transmissions", default_value="true"),
        DeclareLaunchArgument("paused", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("spawn_z", default_value="auto"),
        DeclareLaunchArgument("start_controllers", default_value="true"),
        DeclareLaunchArgument("start_control_nodes", default_value="true"),
        DeclareLaunchArgument("start_gui_tools", default_value="true"),
        DeclareLaunchArgument("start_flipper_joint_gui", default_value="true"),
        DeclareLaunchArgument("start_cloudmap_publisher", default_value="true"),
        DeclareLaunchArgument("load_controller_params", default_value="false"),
        DeclareLaunchArgument("control_yaml", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
