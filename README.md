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

## ROS 2 Nodes & Parameters

This package provides a datalogger server and suitable client APIs in C++ and
Python. Each node is launched through its own launch file, which loads
parameters from `config/config.yaml` (pass a different file with
`config_file:=/path/to/config.yaml`).

### Node: `sas_datalogger`

| Property | Value |
|---|---|
| **Executable** | `sas_datalogger_node.py` |
| **ROS node name** | `sas_datalogger` (set by the `name` launch argument of `sas_datalogger_launch.py`) |
| **Description** | Main datalogger node. Subscribes to `/sas_datalogger/log` and stores received values in memory. When the node is shut down it saves the collected data to a MATLAB-compatible `.mat` file (via `scipy.io.savemat`). |

#### Parameters

The server declares **no ROS parameters** — it subscribes to the fixed topic `/sas_datalogger/log`.

#### Sample launch

```bash
ros2 launch sas_datalogger sas_datalogger_launch.py
```

### Node: `sas_datalogger_gui_node`

| Property | Value |
|---|---|
| **Executable** | `sas_datalogger_gui_node.py` |
| **ROS node name** | `sas_datalogger_gui_node` (set by the `name` launch argument of `sas_datalogger_gui_launch.py`) |
| **Description** | A Qt-based GUI that reads the datalogger's internal dictionary and creates execution-time plots for numeric values. |

#### Parameters

| Parameter | Type | Mandatory / Optional | Default | Purpose |
|---|---|---|---|---|
| `whitelist` | string array | Optional | `[' ']` | List of values to plot. The single-space entry `[' ']` is the package convention for "plot all values" (the node maps it to `None`) |

#### Sample launch

```bash
ros2 launch sas_datalogger sas_datalogger_gui_launch.py
```

To plot only specific values:

```bash
ros2 launch sas_datalogger sas_datalogger_gui_launch.py config_file:=/path/to/config.yaml
```

### Node: `sas_datalogger_client_example` (C++ example client)

| Property | Value |
|---|---|
| **Executable** | `sas_datalogger_client_example` |
| **ROS node name** | `sas_datalogger_client_example` (set by the `name` launch argument of `sas_datalogger_client_cpp_example_launch.py`) |
| **Description** | C++ example client. Publishes matrices, vectors, scalars and strings to the datalogger topic. |

#### Parameters

The C++ example client declares **no ROS parameters**.

#### Sample launch

```bash
ros2 launch sas_datalogger sas_datalogger_client_cpp_example_launch.py
```

### Node: `sas_datalogger_client_example_py_rclpy` (Python example client)

| Property | Value |
|---|---|
| **Executable** | `sas_datalogger_client_example_py.py` |
| **ROS node name** | `sas_datalogger_client_example_py_rclpy` (fixed in the script; the script creates two nodes, so no `name` launch argument is set) |
| **Description** | Python example client. Publishes matrices, vectors, scalars and strings to the datalogger topic. |

#### Parameters

| Parameter | Type | Mandatory / Optional | Default | Purpose |
|---|---|---|---|---|
| `execution_times` | integer | Optional | `5` | How many times the example logs the sample values |

#### Sample launch

```bash
ros2 launch sas_datalogger sas_datalogger_client_python_example_launch.py
```

> [!NOTE]
> The Python example client executable creates **two** nodes (an rclcpp node and
> an rclpy node). The `execution_times` parameter is declared on the rclpy node,
> whose fixed code name is `sas_datalogger_client_example_py_rclpy`; the
> corresponding block in `config/config.yaml` is keyed by that name.
