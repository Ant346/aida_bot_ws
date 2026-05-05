from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odroid_node',
            executable='odroid_driver',
            name='odroid_driver',
            output='screen',
        ),
    ])
