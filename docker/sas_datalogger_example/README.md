# SAS datalogger Docker compose example

Run

```bash
mkdir -p ~/sas_datalogger/docker/sas_datalogger_example
cd ~/sas_datalogger/docker/sas_datalogger_example
curl -OL https://github.com/SmartArmStack/sas_datalogger/tree/jazzy/docker/sas_datalogger_example/compose.yml

docker compose up
```

> [!IMPORTANT]
> To stop, press `Ctrl + C`. Do not forget to remove the containers: 
>
> ```shell
> docker compose down -v
> ```

The saved `.mat` file will be located at `~/sas_datalogger/docker/sas_datalogger_example/logs`.

For a detailed explanation of how to include the SAS datalogger container in your own Docker compose files, check the `compose.yml`.
