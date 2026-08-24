import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the datalogger server node.

    Parameters are loaded from a YAML configuration file. Pass a different
    file with ``config_file:=/path/to/config.yaml``.
    """
    name = LaunchConfiguration('name')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='sas_datalogger'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(get_package_share_directory('sas_datalogger'), 'config', 'config.yaml')
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_datalogger',
            executable='sas_datalogger_node.py',
            name=name,
            parameters=[config_file]
        )
    ])

