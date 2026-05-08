# SAS datalogger Docker compose example

Clone this repository

```bash
cd ~/GitHub &&
git clone https://github.com/SmartArmStack/sas_datalogger.git --recursive
```

and launch the Docker compose

```bash
cd ~/GitHub/sas_datalogger/docker/sas_datalogger_example &&
docker compose up
```

> [!IMPORTANT]
> To stop, press `Ctrl + C`. Do not forget to remove the containers: 
>
> ```shell
> docker compose down -v
> ```

The saved `.mat` file will be located at `~/GitHub/sas_datalogger/docker/sas_datalogger_example/logs`.

For a detailed explanation of how to include the SAS datalogger container in your own Docker compose files, check the `compose.yml`.
