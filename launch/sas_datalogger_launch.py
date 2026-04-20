from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_datalogger',
            executable='sas_datalogger_node.py',
            name='sas_datalogger'
        )
    ])

