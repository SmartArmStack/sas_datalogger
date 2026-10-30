# Copyright (c) 2012-2026 Murilo Marques Marinho
#
#    This file is part of sas_datalogger.
#
#    sas_datalogger is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_datalogger is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_datalogger.  If not, see <https://www.gnu.org/licenses/>.

import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Python example datalogger client.

    This executable creates two nodes (an rclcpp node and an rclpy node), each
    with a fixed name set in the code. The `execution_times` parameter is
    declared on the rclpy node, whose code name is
    ``sas_datalogger_client_example_py_rclpy``. A launch ``name`` is therefore
    not set here (a `__node` remap would be applied ambiguously to one of the
    two nodes); the parameter is loaded from the matching block in the YAML
    configuration file instead. Pass a different file with
    ``config_file:=/path/to/config.yaml``.
    """
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(get_package_share_directory('sas_datalogger'), 'config', 'config.yaml')
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_datalogger',
            executable='sas_datalogger_client_example_py.py',
            parameters=[config_file]
        ),
    ])

