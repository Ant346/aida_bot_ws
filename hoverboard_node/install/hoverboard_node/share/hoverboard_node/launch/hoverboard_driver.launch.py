#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    front_serial_arg = DeclareLaunchArgument(
        'front_serial',
        default_value='0001',
        description='Front hoverboard serial ID'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='0.05',
        description='Serial timeout'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.095',
        description='Wheel radius in meters'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.635',
        description='Wheel base in meters'
    )
    
    wheel_track_arg = DeclareLaunchArgument(
        'wheel_track',
        default_value='0.72',
        description='Wheel track in meters'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='50',
        description='Maximum speed'
    )
    
    test_speed_arg = DeclareLaunchArgument(
        'test_speed',
        default_value='1',
        description='Test speed for individual wheel testing'
    )

    # Create the hoverboard driver node
    hoverboard_driver_node = Node(
        package='hoverboard_node',
        executable='hoverboard_driver',
        name='hoverboard_driver',
        output='screen',
        parameters=[{
            'front_serial': LaunchConfiguration('front_serial'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout': LaunchConfiguration('timeout'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_track': LaunchConfiguration('wheel_track'),
            'max_speed': LaunchConfiguration('max_speed'),
            'test_speed': LaunchConfiguration('test_speed'),
        }]
    )

    return LaunchDescription([
        front_serial_arg,
        baudrate_arg,
        timeout_arg,
        wheel_radius_arg,
        wheel_base_arg,
        wheel_track_arg,
        max_speed_arg,
        test_speed_arg,
        hoverboard_driver_node,
    ]) 