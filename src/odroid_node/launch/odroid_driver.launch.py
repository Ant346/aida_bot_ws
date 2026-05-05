import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('odroid_node')
    params_file = os.path.join(pkg, 'config', 'odroid_driver.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='false',
            description='If true: no serial, log-only'),
        DeclareLaunchArgument(
            'cmd_vel_subscribe_stamped',
            default_value='false',
            description='True if cmd_vel is TwistStamped'),
        Node(
            package='odroid_node',
            executable='odroid_driver',
            name='odroid_driver',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'simulation_mode': ParameterValue(
                        LaunchConfiguration('simulation_mode'), value_type=bool),
                    'cmd_vel_subscribe_stamped': ParameterValue(
                        LaunchConfiguration('cmd_vel_subscribe_stamped'),
                        value_type=bool),
                },
            ],
        ),
    ])
