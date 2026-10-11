"""Start the D435 driver and keep factory camera_info on a side topic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

ARGS = (
    ("camera_namespace", "d435"),
    ("camera_name", "d435"),
    ("device_type", "d435"),
    ("enable_color", "true"),
    ("enable_depth", "false"),
    ("enable_infra1", "false"),
    ("enable_infra2", "false"),
    ("pointcloud.enable", "false"),
    ("align_depth.enable", "false"),
    ("rgb_camera.color_profile", "1280x720x15"),
    ("initial_reset", "false"),
)


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )
    declared = [DeclareLaunchArgument(name, default_value=default) for name, default in ARGS]
    forwarded = {name: LaunchConfiguration(name) for name, _default in ARGS}
    return LaunchDescription(
        declared
        + [
            GroupAction(
                [
                    SetRemap(src="~/color/camera_info", dst="~/color/camera_info_factory"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(rs_launch),
                        launch_arguments=forwarded.items(),
                    ),
                ]
            )
        ]
    )
