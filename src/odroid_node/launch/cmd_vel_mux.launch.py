"""
Mux: teleop vs nav vs shaped -> /cmd_vel.

- Cross (button_cross): teleop <-> auto (cmd_nav).
- Square (button_square): shaped <-> previous teleop/auto choice.

Remap ds4_twist: cmd_vel -> cmd_vel_teleop
Publish shaped commands to cmd_vel_shaped.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odroid_node',
            executable='cmd_vel_mux_node',
            name='cmd_vel_mux',
            output='screen',
            parameters=[{
                'status_topic': '/status',
                'teleop_cmd_vel_topic': '/cmd_vel_teleop',
                'nav_cmd_vel_topic': '/cmd_nav',
                'cmd_vel_out_topic': '/cmd_vel',
                'watchdog_timeout_sec': 2.0,
                'publish_rate_hz': 20.0,
                'initial_navigation_mode': False,
                'allow_nav_when_joy_lost': False,
                'toggle_button_field': 'button_cross',
                'shaped_cmd_vel_topic': '/cmd_vel_shaped',
                'shaped_toggle_button_field': 'button_square',
                'mode_topic': '/cmd_vel_mux/mode',
            }],
        ),
    ])
