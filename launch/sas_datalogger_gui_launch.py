from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    whitelist = LaunchConfiguration('whitelist')

    return LaunchDescription([
        DeclareLaunchArgument(
            'whitelist',
            default_value="[' ',]"
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_datalogger',
            executable='sas_datalogger_gui_node.py',
            name='sas_datalogger',
            parameters=[{
                'whitelist': whitelist
            }]
        )
    ])

