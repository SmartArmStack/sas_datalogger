#!/bin/bash

cd /root/sas_datalogger_devel
ls .
colcon build
source install/setup.bash
timeout -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_client_cpp_example_launch.py
timeout -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_client_python_example_launch.py
ros2 run sas_datalogger sas_datalogger_client_example_result_check.py