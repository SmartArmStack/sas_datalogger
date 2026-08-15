FROM murilomarinho/sas:jazzy
LABEL authors="murilomarinho"
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

RUN sudo apt-get update && sudo apt-get upgrade -y

COPY docker/install.sh /root/sas_datalogger/
RUN cd /root/sas_datalogger \
    && chmod +x install.sh \
    && . install.sh

RUN sudo sudo apt-get remove -y ros-jazzy-sas-datalogger
RUN mkdir -p /root/sas_datalogger_devel/src/
COPY . /root/sas_datalogger_devel/src/sas_datalogger