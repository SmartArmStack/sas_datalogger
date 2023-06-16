# sas_datalogger

Log data in your CPP and Python nodes through `ROS2` into a `.mat` file.

- [X] `rclcpp` Native implementation.
- [ ] `rclpy` TODO as a `pybind11` wrapper.

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

2. Press `CTRL+C` to finish both nodes. The `.mat` file will be saved in the current directory, i.e. the one in which you ran the launch file.

3. Check the values stored in the example log file as follows. The log filename will depend on the current timestamp.

```commandLine
ros2 run sas_datalogger sas_datalogger_client_example_result_check.py sas_log_2023_06_16_12_40_25.mat
```

#### Main points in the example's source code

1. Use the appropriate header

https://github.com/SmartArmStack/sas_datalogger/blob/f8654d0803efc09cde006feec5a640e8b1d459b3/src/examples/sas_datalogger_client_example.cpp#L27

2. Instantiate an object from the datalogger class

https://github.com/SmartArmStack/sas_datalogger/blob/f8654d0803efc09cde006feec5a640e8b1d459b3/src/examples/sas_datalogger_client_example.cpp#L53

3. Wait for connection with the `sas_datalogger_node.py`

https://github.com/SmartArmStack/sas_datalogger/blob/f8654d0803efc09cde006feec5a640e8b1d459b3/src/examples/sas_datalogger_client_example.cpp#L58-L62

4. Log different types of data as follows

https://github.com/SmartArmStack/sas_datalogger/blob/f8654d0803efc09cde006feec5a640e8b1d459b3/src/examples/sas_datalogger_client_example.cpp#L67-L78               
