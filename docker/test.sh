#!/bin/bash

cd /root/sas_datalogger_devel || exit 1
ls .
colcon build
source install/setup.bash
colcon test
colcon test-result || exit 1
sudo apt-get update -qq && sudo apt-get install -y -qq libasan8 libubsan1
colcon build --build-base build_san --install-base install_san --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -O1 -g" -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined" -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address,undefined" || exit 1
source install_san/setup.bash
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 colcon test --build-base build_san
colcon test-result --test-result-base build_san --verbose | grep -E "Summary: [1-9][0-9]* tests, 0 errors, 0 failures" || exit 1
timeout --foreground -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_launch.py &
timeout -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_client_cpp_example_launch.py
timeout --foreground -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_launch.py &
timeout -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_client_python_example_launch.py
ros2 run sas_datalogger sas_datalogger_client_example_result_check.py

sudo apt-get install -y xvfb
sudo /usr/bin/Xvfb :99 -screen 0 1280x1024x24 &
timeout --foreground -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_client_cpp_example_launch.py &
timeout -s INT $TIMEOUT ros2 launch sas_datalogger sas_datalogger_gui_launch.py