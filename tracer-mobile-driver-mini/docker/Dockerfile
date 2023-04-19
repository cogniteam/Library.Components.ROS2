FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update && apt-get upgrade -y && apt-get -y install python3-pip && apt-get install -y --no-install-recommends apt-utils

# Install dependencies

RUN apt-get install build-essential git cmake libasio-dev -y
RUN apt install iproute2 -y

# Cloning this repository into your workspace
RUN mkdir -p ~/tracer_driver_ws/src
WORKDIR /root/tracer_driver_ws/src
RUN git clone https://github.com/westonrobot/ugv_sdk.git
RUN git clone https://github.com/agilexrobotics/tracer_ros2.git



# Rebuild your workspace
WORKDIR /root/tracer_driver_ws
RUN . /opt/ros/foxy/setup.sh && colcon build

WORKDIR /root/tracer_driver_ws/src/ugv_sdk/scripts
RUN bash setup_can2usb.bash

WORKDIR /root/tracer_driver_ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


