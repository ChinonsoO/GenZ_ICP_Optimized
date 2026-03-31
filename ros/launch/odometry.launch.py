# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
# Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


UNSET_OVERRIDE = "__unset__"
PARAMETER_DEFAULTS = {
    "deskew": "false",
    "max_range": "100.0",
    "min_range": "0.3",
    "voxel_size": "0.3",
    "map_cleanup_radius": "100.0",
    "desired_num_voxelized_points": "2000",
    "planarity_threshold": "0.2",
    "max_points_per_voxel": "1",
    "max_num_iterations": "100",
    "convergence_criterion": "0.0001",
    "initial_threshold": "2.0",
    "min_motion_th": "0.1",
}


def _coerce_launch_value(raw_value):
    parsed_value = yaml.safe_load(raw_value)
    return raw_value if parsed_value is None else parsed_value


def load_config_parameters(config_file: str) -> dict:
    if not config_file:
        return {}

    config_path = Path(config_file).expanduser()
    if not config_path.is_absolute():
        installed_config_dir = Path(__file__).resolve().parents[1] / "config"
        candidate = installed_config_dir / config_path.name
        if candidate.exists():
            config_path = candidate
        else:
            config_path = (Path(__file__).resolve().parent / config_path).resolve()

    config_payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(config_payload, dict):
        raise RuntimeError(f"GenZ config file must contain a flat mapping: {config_path}")
    return config_payload


def resolve_node_parameters(launch_values: dict) -> dict:
    config_file = str(launch_values.get("config_file", "indoor.yaml"))
    has_config_file = bool(config_file)
    node_parameters = load_config_parameters(config_file)
    node_parameters.update(
        {
            "odom_frame": str(launch_values["odom_frame"]),
            "base_frame": str(launch_values["base_frame"]),
            "publish_odom_tf": _coerce_launch_value(str(launch_values["publish_odom_tf"])),
            "visualize": _coerce_launch_value(str(launch_values["visualize"])),
        }
    )

    for parameter_name, default_value in PARAMETER_DEFAULTS.items():
        raw_value = str(launch_values[parameter_name])
        if raw_value == UNSET_OVERRIDE:
            if not has_config_file:
                node_parameters[parameter_name] = _coerce_launch_value(default_value)
            continue
        node_parameters[parameter_name] = _coerce_launch_value(raw_value)

    return node_parameters


def _create_runtime_actions(context, *args, **kwargs):
    current_pkg = FindPackageShare("genz_icp").perform(context)
    launch_values = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "topic",
            "bagfile",
            "visualize",
            "odom_frame",
            "base_frame",
            "publish_odom_tf",
            "deskew",
            "max_range",
            "min_range",
            "voxel_size",
            "map_cleanup_radius",
            "desired_num_voxelized_points",
            "planarity_threshold",
            "max_points_per_voxel",
            "max_num_iterations",
            "convergence_criterion",
            "initial_threshold",
            "min_motion_th",
            "config_file",
        )
    }
    node_parameters = resolve_node_parameters(launch_values)

    return [
        Node(
            package="genz_icp",
            executable="odometry_node",
            name="odometry_node",
            output="screen",
            remappings=[("pointcloud_topic", launch_values["topic"])],
            parameters=[node_parameters],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output={"both": "log"},
            arguments=["-d", str(Path(current_pkg) / "rviz" / "genz_icp_ros2.rviz")],
            condition=IfCondition(LaunchConfiguration("visualize")),
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", LaunchConfiguration("bagfile")],
            output="screen",
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration("bagfile"), "' != ''"])
            ),
        ),
    ]


def generate_launch_description():
    current_pkg = FindPackageShare("genz_icp")
    return LaunchDescription(
        [
            # ROS 2 parameters
            DeclareLaunchArgument("topic", description="sensor_msg/PointCloud2 topic to process"),
            DeclareLaunchArgument("bagfile", default_value=""),
            DeclareLaunchArgument("visualize", default_value="true"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value=""),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            # GenZ-ICP parameters
            DeclareLaunchArgument("deskew", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("max_range", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("min_range", default_value=UNSET_OVERRIDE),
            # This thing is still not suported: https://github.com/ros2/launch/issues/290#issuecomment-1438476902
            #  DeclareLaunchArgument("voxel_size", default_value=None),
            DeclareLaunchArgument("voxel_size", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("map_cleanup_radius", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("desired_num_voxelized_points", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("planarity_threshold", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("max_points_per_voxel", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("max_num_iterations", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("convergence_criterion", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("initial_threshold", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument("min_motion_th", default_value=UNSET_OVERRIDE),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution([current_pkg, "config", "indosor.yaml"]),
            ),
            OpaqueFunction(function=_create_runtime_actions),
        ]
    )
