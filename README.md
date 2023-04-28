# sas_datalogger

Log data in your CPP and Python nodes through `ROS2` into a `.mat` file.

- [X] `rclcpp` Native implementation.
- [ ] `rclpy` TODO as a `pybind11` wrapper.

## Main goodies

### Node

Call with `ros2 run sas_datalogger <NODE_NAME>`.

| Node name                             | Description                                                                                                                   |
|------------------------------------|-------------------------------------------------------------------------------------------------------------------------------|
| `sas_datalogger_node.py`              | A convenience wrapper contaning all conversion headers.                                                                       |

#### Example

Running the example:

```commandline
ros2 launch sas_conversions sas_datalogger_client_example.py
```

##### Main points in the example's source code

Use the appropriate header

#include <sas_datalogger/sas_datalogger_client.hpp>

So that you have access to the datalogger class

https://github.com/SmartArmStack/sas_datalogger/blob/f8654d0803efc09cde006feec5a640e8b1d459b3/src/examples/sas_datalogger_client_example.cpp#L53

Then, you can log different types of data as follows

https://github.com/SmartArmStack/sas_datalogger/blob/f8654d0803efc09cde006feec5a640e8b1d459b3/src/examples/sas_datalogger_client_example.cpp#L67-78               |
