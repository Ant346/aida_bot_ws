"""Publish agro_cad and open RViz. Meshes stay on the mounted package tree."""

from pathlib import Path

from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESCRIPTION_ROOT = Path("/opt/agrobot_description")
URDF_PATH = DESCRIPTION_ROOT / "urdf" / "agrobot.urdf"
PACKAGE_PREFIX = "package://agrobot_description/"
FILE_PREFIX = f"file://{DESCRIPTION_ROOT}/"


def _robot_description() -> str:
    text = URDF_PATH.read_text(encoding="utf-8")
    return text.replace(PACKAGE_PREFIX, FILE_PREFIX)


def generate_launch_description() -> LaunchDescription:
    robot_description = ParameterValue(_robot_description(), value_type=str)
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["python3", "/workspace/rviz_configs/zero_joint_states.py"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", "/workspace/rviz_configs/agrobot.rviz"],
                output="screen",
            ),
        ]
    )
