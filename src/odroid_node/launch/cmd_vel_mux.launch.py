"""
Mux: /cmd_vel_teleop (ds4_twist) vs /cmd_nav (autonomy) -> /cmd_vel.

Remap ds4_twist to publish to cmd_vel_teleop, e.g. when starting ds4_twist:
  ros2 run ds4_driver ds4_twist_node.py --ros-args -r cmd_vel:=cmd_vel_teleop

Or add the remap in ds4_twist.launch.xml for the ds4_twist node only.
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
                'mode_topic': '/cmd_vel_mux/mode',
            }],
        ),
    ])
