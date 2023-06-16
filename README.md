# sas_datalogger

Log data in your CPP and Python nodes through `ROS2` into a `.mat` file.

- [X] `rclcpp`. Native implementation.
- [X] `rclpy` with a `pybind11` wrapper.

## Main goodies

### Node

Call with `ros2 run sas_datalogger <NODE_NAME>`.

| Node name                             | Description                                                                                                                   |
|---------------------------------------|-------------------------------------------------------------------------------------------------------------------------------|
| `sas_datalogger_node.py`              | A convenience wrapper containing all conversion headers.                                                                       |

### Example

1. Run the example

```commandline
ros2 launch sas_conversions sas_datalogger_client_example.py
```

2. Press `CTRL+C` to end both nodes. The `.mat` file will be saved in the current directory, i.e. the one in which you ran the launch file.

3. Check the values stored in the example log file as follows. The log filename will depend on the current timestamp.

```commandLine
ros2 run sas_datalogger sas_datalogger_client_example_result_check.py sas_log_2023_06_16_12_40_25.mat
```

#### CPP Usage

Refer to the example `src/examples/sas_datalogger_client_example.cpp`.

https://github.com/SmartArmStack/sas_datalogger/blob/c18667d55c1293dbfcb3491b4e17e2ba095620dc/src/examples/sas_datalogger_client_example.cpp#L25-L102
    
#### Python Usage

Refer to the example `scripts/sas_datalogger_client_example_py.py`.

https://github.com/SmartArmStack/sas_datalogger/blob/c18667d55c1293dbfcb3491b4e17e2ba095620dc/scripts/sas_datalogger_client_example_py.py#L27-L82
