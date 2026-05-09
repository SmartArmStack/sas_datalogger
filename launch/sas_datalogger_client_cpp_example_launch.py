from launch import LaunchDescription
from launch_ros.actions import Node

# Run the example "src/examples/sas_datalogger_client_example.cpp"
def generate_launch_description():
    return LaunchDescription([
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_datalogger',
            executable='sas_datalogger_client_example',
            name='sas_datalogger_client_example'
        ),
    ])

