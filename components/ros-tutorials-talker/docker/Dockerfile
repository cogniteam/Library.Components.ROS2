FROM ros:foxy


# USE BASH
SHELL ["/bin/bash", "-c"]


RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils

COPY ./talker /talker_ws/src/talker
 
WORKDIR /talker_ws

RUN apt-get install ros-foxy-std-msgs -y && \
    rm /var/lib/apt/lists/* -rf

RUN . /opt/ros/foxy/setup.sh && colcon build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["ros2", "run" ,"talker", "talker_node"]

