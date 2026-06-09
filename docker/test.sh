#!/bin/bash

cd /root/sas_datalogger_devel || exit 1
ls .
colcon build
source install/setup.bash
timeout --foreground -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_launch.py &
timeout -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_client_cpp_example_launch.py
timeout --foreground -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_launch.py &
timeout -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_client_python_example_launch.py
ros2 run sas_datalogger sas_datalogger_client_example_result_check.py

sudo apt-get install -y xvfb
sudo /usr/bin/Xvfb :99 -screen 0 1280x1024x24 &
timeout -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_gui_launch.py