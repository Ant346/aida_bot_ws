from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    use_video = LaunchConfiguration("use_video")
    video_file = LaunchConfiguration("video_file")
    visualize = LaunchConfiguration("visualize")
    control_enabled = LaunchConfiguration("control_enabled")

    default_config = PathJoinSubstitution(
        [FindPackageShare("greenhouse_pipe_rail_nav"), "config", "default.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("use_video", default_value="false"),
            DeclareLaunchArgument("video_file", default_value=""),
            DeclareLaunchArgument("visualize", default_value="true"),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            Node(
                package="greenhouse_pipe_rail_nav",
                executable="video_image_publisher",
                name="pipe_rail_video_image_publisher",
                condition=IfCondition(use_video),
                parameters=[
                    {
                        "video_file": video_file,
                        "image_topic": "/camera/color/image_raw",
                        "loop": True,
                        "fps": 30.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="greenhouse_pipe_rail_nav",
                executable="rail_autodock_node",
                name="greenhouse_pipe_rail_autodock",
                parameters=[
                    config_file,
                    {
                        "visualize": visualize,
                        "control_enabled": control_enabled,
                    },
                ],
                output="screen",
            ),
        ]
    )
