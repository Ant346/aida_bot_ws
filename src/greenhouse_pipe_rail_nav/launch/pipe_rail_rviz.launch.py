"""RViz with the agro_cad model mounted at /data/agro_cad."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

AGRO_ROOT = Path("/data/agro_cad/agrobot_description")


def _robot_description() -> str:
    text = (AGRO_ROOT / "urdf" / "agrobot.urdf").read_text(encoding="utf-8")
    return text.replace("package://agrobot_description/", f"file://{AGRO_ROOT}/")


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("greenhouse_pipe_rail_nav"), "rviz", "pipe_rail.rviz"]
    )
    robot_description = ParameterValue(_robot_description(), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("robot_description", "/agrobot/robot_description")],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("robot_description", "/agrobot/robot_description")],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
