FROM murilomarinho/sas:jazzy
LABEL authors="murilomarinho"
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

# Update
RUN sudo apt-get update && sudo apt-get upgrade -y

# Copy files from this repository to the image
WORKDIR /app

COPY ./include /app/sas_datalogger/include
COPY ./src /app/sas_datalogger/src

COPY ./CMakeLists.txt /app/sas_datalogger/CMakeLists.txt
COPY ./package.xml /app/sas_datalogger/package.xml

COPY ./scripts /app/sas_datalogger/scripts
COPY ./launch /app/sas_datalogger/launch

COPY ./pybind11 /app/sas_datalogger/pybind11
COPY ./sas_datalogger /app/sas_datalogger/sas_datalogger

# Build and colcon build
RUN mkdir -p /app/sas_datalogger/build
RUN cd /app/sas_datalogger/build && cmake .. && make && make install
RUN ldconfig

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; cd /app/sas_datalogger/; colcon build && source install/setup.bash"