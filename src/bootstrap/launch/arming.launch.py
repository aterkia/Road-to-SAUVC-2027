"""Enable explicit arm/disarm services without starting motion or a mission."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bench = Path(get_package_share_directory('bootstrap')) / 'launch' / 'bench.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bench)),
            launch_arguments={'arming_enabled': 'true'}.items(),
        ),
    ])
