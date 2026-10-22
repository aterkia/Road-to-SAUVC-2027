from pathlib import Path
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mock', 'hardware'):
        raise RuntimeError('mode must be mock or hardware')
    share = Path(get_package_share_directory('bootstrap'))
    config = LaunchConfiguration('config').perform(context)
    if not config:
        config = str(share / 'config' / ('autonomous.yaml' if mode == 'hardware' else 'auv.yaml'))
    if mode == 'hardware':
        bench = share / 'config' / 'bench.yaml'
        with bench.open(encoding='utf-8') as stream:
            base = yaml.safe_load(stream)
        with open(config, encoding='utf-8') as stream:
            settings = yaml.safe_load(stream)
        helm = settings.get('helm', {}).get('ros__parameters', {})
        depth = {**base.get('helm', {}).get('ros__parameters', {}), **helm}
        if depth.get('firmware_version') in (None, '', 'unverified') or not depth.get('pressure_is_depth_sensor'):
            raise RuntimeError('Hardware requires verified firmware and calibrated water-pressure configuration; edit autonomous.yaml')
        source = helm.get('source_system', 255)
        if type(source) is not int or not 1 <= source <= 255:
            raise ValueError('source_system must be an integer in 1..255')
        endpoint = settings.get('/mavros_node', {}).get('ros__parameters', {}).get(
            'fcu_url', base['/mavros_node']['ros__parameters']['fcu_url'])
        camera_required = 'APPROACH_GATE' in settings.get('orchestrator', {}).get(
            'ros__parameters', {}).get('tasks', [])
        camera_launch = settings.get('bootstrap', {}).get('ros__parameters', {}).get('camera_launch_file', '')
        drivers = []
        if camera_launch:
            driver = Path(camera_launch).expanduser()
            if not driver.is_file():
                raise RuntimeError(f'Camera driver launch not found: {driver}')
            drivers.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver))))
        return drivers + [
            Node(package='mavros', executable='mavros_node', parameters=[str(bench), config, {
                'fcu_url': endpoint, 'gcs_url': '', 'system_id': source,
                'plugin_allowlist': ['sys_status', 'sys_time', 'imu', 'command', 'param', 'manual_control'],
                'plugin_denylist': ['*'],
            }], output='screen', on_exit=Shutdown(reason='MAVROS exited; autonomous mission stopped')),
            Node(package='helm', executable='helm_mavros_monitor', name='helm',
                 parameters=[str(bench), config, {'control_enabled': True, 'arming_enabled': True}],
                 output='screen', on_exit=Shutdown(reason='helm exited; autonomous mission stopped')),
            *[Node(package=name, executable=f'{name}_node', name=name,
                   parameters=[config, {'vehicle_mode': 'hardware',
                       **({'require_camera_on_start': camera_required} if name == 'wayfinder' else {})}],
                   output='screen',
                   on_exit=Shutdown(reason=f'{name} exited; autonomous mission stopped'))
              for name in ('wayfinder', 'spectra', 'orchestrator')],
        ]
    return [
        Node(
            package=name, executable=f'{name}_node', name=name, parameters=[config], output='screen'
        )
        for name in ('helm', 'wayfinder', 'spectra', 'orchestrator')
    ]


def generate_launch_description():
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
            DeclareLaunchArgument('config', default_value=''),
            OpaqueFunction(function=launch_nodes),
        ]
    )
