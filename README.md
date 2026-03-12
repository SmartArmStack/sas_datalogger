# sas_datalogger

Log data through `ROS2` into a `.mat`-compliant file.

## Main goodies

### Node

Call with `ros2 run sas_datalogger <NODE_NAME>`.

| Node name                                             | Description                                                                 |
|-------------------------------------------------------|-----------------------------------------------------------------------------|
| `sas_datalogger_node.py`                              | The main node that will store the data received through specialised topics. |
| `#include <sas_datalogger/sas_datalogger_client.hpp>` | The `DataloggerClient` that must be used for `cpp` binaries.                |
| `from sas_datalogger import DataloggerClient`         | The `DataloggerClient` that must be used in `Python` scripts.               |

### Example

```console
cd docker
docker compose up
```

#### CPP Usage

Refer to the example `src/examples/sas_datalogger_client_example.cpp`.

https://github.com/SmartArmStack/sas_datalogger/blob/c18667d55c1293dbfcb3491b4e17e2ba095620dc/src/examples/sas_datalogger_client_example.cpp#L25-L102
    
#### Python Usage

Refer to the example `scripts/sas_datalogger_client_example_py.py`.

https://github.com/SmartArmStack/sas_datalogger/blob/c18667d55c1293dbfcb3491b4e17e2ba095620dc/scripts/sas_datalogger_client_example_py.py#L27-L82
