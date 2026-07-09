#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    simulator = LaunchConfiguration("simulator").perform(context).lower()

    common_args = {
        "worldfile": LaunchConfiguration("worldfile"),
        "arena_yaml": LaunchConfiguration("arena_yaml"),
        "simsetting_yaml": LaunchConfiguration("simsetting_yaml"),
        "spawn_arena": LaunchConfiguration("spawn_arena"),
        "gui": LaunchConfiguration("gui"),
        "paused": LaunchConfiguration("paused"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "norobot": "true",
        "start_gui_tools": "false",
        "start_flipper_joint_gui": "false",
        "start_cloudmap_publisher": "true",
    }

    if simulator in {"ignition", "ign", "gz"}:
        launch_file = crawler_gazebo_share / "launch" / "gazebo_ignition.launch.py"
        launch_args = {
            **common_args,
            "render_engine_gui": LaunchConfiguration("render_engine_gui"),
            "ign_partition": LaunchConfiguration("ign_partition"),
        }
    elif simulator in {"gazebo11", "classic", "gazebo"}:
        launch_file = crawler_gazebo_share / "launch" / "gazebo.launch.py"
        launch_args = {
            **common_args,
            "start_controllers": "false",
            "start_control_nodes": "false",
        }
    else:
        raise RuntimeError("simulator must be one of: ignition, gazebo11")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)),
            launch_arguments=launch_args.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("simulator", default_value="ignition"),
        DeclareLaunchArgument("worldfile", default_value="base_fields.world"),
        DeclareLaunchArgument("arena_yaml", default_value="benchmark.yaml"),
        DeclareLaunchArgument("simsetting_yaml", default_value="simsetting.yaml"),
        DeclareLaunchArgument("spawn_arena", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("render_engine_gui", default_value="ogre"),
        DeclareLaunchArgument("ign_partition", default_value="auto"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
