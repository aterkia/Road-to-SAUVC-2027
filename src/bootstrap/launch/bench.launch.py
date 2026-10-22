"""MAVROS + helm telemetry, with opt-in arm/disarm; no mission or motion."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    arming = LaunchConfiguration('arming_enabled').perform(context)
    if arming not in ('true', 'false'):
        raise ValueError('arming_enabled must be true or false')
    arming_enabled = arming == 'true'
    config = LaunchConfiguration('config').perform(context)
    with open(config, encoding='utf-8') as stream:
        settings = yaml.safe_load(stream)

    endpoint = LaunchConfiguration('fcu_url').perform(context)
    if not endpoint:
        endpoint = settings.get('/mavros_node', {}).get('ros__parameters', {}).get('fcu_url', '')
    if not isinstance(endpoint, str) or not endpoint.strip():
        raise RuntimeError(
            'bench requires an explicit fcu_url (serial or UDP); no hardware endpoint is assumed'
        )

    # RC, servo and motion/setpoint plugins remain excluded in both profiles.
    plugins = ['sys_status', 'sys_time', 'imu']
    if arming_enabled:
        plugins.append('command')
    return [
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            on_exit=Shutdown(reason='MAVROS exited'),
            parameters=[
                config,
                {
                    'fcu_url': endpoint,
                    'gcs_url': '',
                    'plugin_allowlist': plugins,
                    'plugin_denylist': ['*'],
                },
            ],
        ),
        Node(
            package='helm',
            executable='helm_mavros_monitor',
            name='helm',
            parameters=[config, {'arming_enabled': arming_enabled}],
            output='screen',
            on_exit=Shutdown(reason='helm monitor exited'),
        ),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('bootstrap')) / 'config' / 'bench.yaml'
    return LaunchDescription(
        [
            DeclareLaunchArgument('config', default_value=str(config)),
            DeclareLaunchArgument('fcu_url', default_value=''),
            DeclareLaunchArgument('arming_enabled', default_value='false'),
            OpaqueFunction(function=launch_nodes),
        ]
    )
