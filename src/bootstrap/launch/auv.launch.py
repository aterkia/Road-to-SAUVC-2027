from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode != 'mock':
        raise RuntimeError('Only mock is implemented; hardware requires a verified adapter')
    config = LaunchConfiguration('config').perform(context)
    return [
        Node(
            package=name, executable=f'{name}_node', name=name, parameters=[config], output='screen'
        )
        for name in ('helm', 'wayfinder', 'spectra', 'orchestrator')
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('bootstrap')) / 'config' / 'auv.yaml'
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                'RMW_IMPLEMENTATION', os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
            ),
            SetEnvironmentVariable(
                'FASTDDS_BUILTIN_TRANSPORTS',
                os.environ.get(
                    'FASTDDS_BUILTIN_TRANSPORTS',
                    'LARGE_DATA?max_msg_size=1MB&sockets_size=2MB&non_blocking=true',
                ),
            ),
            DeclareLaunchArgument('mode', default_value='mock'),
            DeclareLaunchArgument('config', default_value=str(config)),
            OpaqueFunction(function=launch_nodes),
        ]
    )
