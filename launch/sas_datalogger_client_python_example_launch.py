from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    execution_times = LaunchConfiguration('execution_times')

    return LaunchDescription([
        DeclareLaunchArgument(
            'execution_times',
            default_value='5'
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_datalogger',
            executable='sas_datalogger_client_example_py.py',
            name='sas_datalogger_client_example',
            parameters=[{
                "execution_times": execution_times
            }]
        ),
    ])

