#!/usr/bin/env python3
"""RealSense: color + aligned depth point cloud. Topics usually /camera/color/image_raw etc."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rs_share = get_package_share_directory('realsense2_camera')
    rs_launch = os.path.join(rs_share, 'launch', 'rs_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
            }.items(),
        ),
    ])
