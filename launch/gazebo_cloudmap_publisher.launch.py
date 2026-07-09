#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    worldfile = LaunchConfiguration("worldfile").perform(context)

    actions = [
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            f"{crawler_gazebo_share / 'gazebo_model' / 'model'}",
        )
    ]

    if _as_bool(LaunchConfiguration("launch_gazebo").perform(context)):
        actions.append(
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    "-s", "libgazebo_ros_init.so",
                    "-s", "libgazebo_ros_factory.so",
                    "-s", "libgazebo_ros_state.so",
                    worldfile,
                ],
                output="screen",
            )
        )
        if _as_bool(LaunchConfiguration("gazebo_gui").perform(context)):
            actions.append(ExecuteProcess(cmd=["gzclient"], output="screen"))

    actions.extend([
        Node(
            package="crawler_gazebo",
            executable="gazebo_to_octomap_publisher",
            name="gazebo_to_octomap_publisher",
            output="screen",
            parameters=[{
                "resolution": float(LaunchConfiguration("resolution").perform(context)),
                "publish_period": float(LaunchConfiguration("publish_period").perform(context)),
            }],
            remappings=[
                ("mesh_marker", "/gazebo/mesh_marker"),
            ],
        ),
        Node(
            package="crawler_gazebo",
            executable="voxel_overhang_removal",
            name="voxel_overhang_removal",
            output="screen",
            parameters=[{
                "input_topic": "/octomap_pointcloud",
                "output_topic": "/octomap_pointcloud/filtering",
                "voxel_size": 0.05,
                "height_mode": "max",
                "durability": "transient_local",
            }],
        ),
    ])
    return actions


def generate_launch_description():
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    return LaunchDescription([
        DeclareLaunchArgument("launch_gazebo", default_value="false"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("worldfile", default_value=str(crawler_gazebo_share / "world" / "base_fields.world")),
        DeclareLaunchArgument("resolution", default_value="0.025"),
        DeclareLaunchArgument("publish_period", default_value="1.0"),
        OpaqueFunction(function=_launch_setup),
    ])
