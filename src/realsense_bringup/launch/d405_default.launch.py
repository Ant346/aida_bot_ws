#!/usr/bin/env python3
"""RealSense D405 on its own namespace so it does not take the D435 device."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rs_share = get_package_share_directory('realsense2_camera')
    rs_launch = os.path.join(rs_share, 'launch', 'rs_launch.py')

    serial = os.environ.get('REALSENSE_D405_SERIAL', '').strip()
    if serial.startswith('_'):
        serial = serial[1:]
    # ROS 2 turns a digits-only CLI value into an int. The driver strips one leading '_'.
    serial_arg = f'_{serial}' if serial else ''

    # This D405 is on a USB 2.0 port (480M). 848x480x15 fits that link; use a USB 3 port for 30 fps.
    depth_profile = os.environ.get('REALSENSE_D405_DEPTH_PROFILE', '848x480x15').strip() or '848x480x15'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_namespace': 'd405',
                'camera_name': 'd405',
                'device_type': 'd405',
                'serial_no': serial_arg,
                'enable_color': 'true',
                'enable_depth': 'true',
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
                'depth_module.depth_profile': depth_profile,
                'depth_module.color_profile': depth_profile,
            }.items(),
        ),
    ])
