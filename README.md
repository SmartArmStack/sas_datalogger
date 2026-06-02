# sas_datalogger

> [!TIP]
> Repository for this module: https://github.com/SmartArmStack/sas_datalogger <br />
> More information about SmartArmStack is available at https://smartarmstack.github.io/.

## Quick start

```bash
mkdir -p ~/sas_datalogger/docker/sas_datalogger_example
cd ~/sas_datalogger/docker/sas_datalogger_example
curl -OL curl -OL https://raw.githubusercontent.com/SmartArmStack/sas_datalogger/refs/heads/jazzy/docker/sas_datalogger_example/compose.yml

docker compose up
```

> [!IMPORTANT]
> To stop, press `Ctrl + C`. Do not forget to remove the containers: 
>
> ```shell
> docker compose down -v
> ```

The saved `.mat` file will be located at `~/sas_datalogger/docker/sas_datalogger_example/logs`.

## ROS 2 Nodes and Launch Files

This package provides a datalogger server and suitable client APIs in C++ and Python.

### sas_datalogger_node

The main datalogger node subscribes to `/sas_datalogger/log` and stores received values in memory.
When the node is shut down it saves the collected data to a MATLAB-compatible `.mat` file (via `scipy.io.savemat`). 

Recommended use is through the launch file. The server must be launched separately.

```bash
ros2 launch sas_datalogger sas_datalogger_launch.py
```

### sas_datalogger_gui_node

A Qt-based GUI that reads the datalogger's internal dictionary and creates
 execution-time plots for numeric values. 

Recommended use is through the launch file. The server must be launched separately.

```bash
ros2 launch sas_datalogger sas_datalogger_gui_launch.py
```

### Example client scripts

The package includes simple example clients that publish matrices, vectors,
scalars and strings to the datalogger topic.

- `scripts/sas_datalogger_client_example_py.py` — Python example client.
- `src/examples/sas_datalogger_client_example.cpp` (binary: `sas_datalogger_client_example`) — C++ example client.
- `scripts/sas_datalogger_client_example_result_check.py` — opens and inspects the generated `.mat` file using `scipy.io.loadmat`.

Recommended use is through the launch file. The server must be launched separately.

```bash
ros2 launch sas_datalogger sas_datalogger_client_python_example_launch.py
```

Recommended use is through the launch file. The server must be launched separately.

```bash
ros2 launch sas_datalogger sas_datalogger_client_cpp_example_launch.py
```

