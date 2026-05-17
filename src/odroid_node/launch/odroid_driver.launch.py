import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _env_bool(name: str, default: bool = False) -> bool:
    v = os.environ.get(name, '').strip().lower()
    if not v:
        return default
    return v in ('1', 'true', 'yes', 'on', 'y')


def _env_str(name: str) -> str:
    return os.environ.get(name, '').strip()


def generate_launch_description():
    pkg = get_package_share_directory('odroid_node')
    defaults_path = os.path.join(pkg, 'config', 'odroid_driver.yaml')
    overlays = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool),
        'simulation_mode': ParameterValue(
            LaunchConfiguration('simulation_mode'), value_type=bool),
        'cmd_vel_subscribe_stamped': ParameterValue(
            LaunchConfiguration('cmd_vel_subscribe_stamped'),
            value_type=bool),
    }

    # One-bus override: rear CAN bus is dead (адаптер отвалился, или нет порта).
    # Включается через ENV ROBOT_SINGLE_CAN=true в docker-compose. В этом случае:
    #   - can_interface_rear="" → узел не пытается открыть can1
    #   - axis_id_rl=-1, axis_id_rr=-1 → задние моторы не получают команд
    # Передняя ось (FL/FR) едет с тем же `can_interface` (по умолчанию can0).
    # Поворот по \omega_z кинематически рассчитан под 4 колеса; с двумя поедет
    # «прямо» нормально, повороты будут с уводом — это ожидаемо.
    single_can = _env_bool('ROBOT_SINGLE_CAN', False)
    can_interface_env = _env_str('CAN_INTERFACE')
    can_interface_rear_env = _env_str('CAN_INTERFACE_REAR')

    if can_interface_env:
        overlays['can_interface'] = can_interface_env
    if single_can:
        overlays['can_interface_rear'] = ''
        overlays['axis_id_rl'] = -1
        overlays['axis_id_rr'] = -1
    elif can_interface_rear_env:
        # Только непустое значение переопределяет YAML. Иначе compose часто передаёт
        # CAN_INTERFACE_REAR="" по умолчанию — это не должно затирать can1 из yaml.
        overlays['can_interface_rear'] = can_interface_rear_env

    params = []
    if os.path.isfile(defaults_path):
        params.append(defaults_path)
    params.append(overlays)

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
        DeclareLaunchArgument(
            'enable_cmd_vel_mux',
            default_value='true',
            description='Run cmd_vel mux (DS4: cross=teleop/nav, square=cmd_vel_shaped)'),
        LogInfo(
            msg=(
                '[odroid_driver] using ' + defaults_path
                if os.path.isfile(defaults_path)
                else (
                    '[odroid_driver] YAML not in install/ — using code defaults; '
                    'rebuild: colcon build --packages-select odroid_node'
                )
            ),
        ),
        LogInfo(
            msg='[odroid_driver] ROBOT_SINGLE_CAN=true → rear CAN disabled, RL/RR axes off',
            condition=IfCondition('true' if single_can else 'false'),
        ),
        Node(
            package='odroid_node',
            executable='odroid_driver',
            name='odroid_driver',
            output='screen',
            parameters=params,
        ),
        Node(
            package='odroid_node',
            executable='cmd_vel_mux_node',
            name='cmd_vel_mux',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_cmd_vel_mux')),
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
