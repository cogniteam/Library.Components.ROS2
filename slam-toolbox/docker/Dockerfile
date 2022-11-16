FROM ros:eloquent-ros-base-bionic


# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils

#install Nav2 packages
RUN apt install ros-eloquent-navigation2 -y 
RUN apt install ros-eloquent-nav2-bringup -y

#slam_toolbox
RUN apt install ros-eloquent-slam-toolbox -y

WORKDIR colcon_ws/

RUN source /opt/ros/eloquent/setup.bash \
    && colcon build --symlink-install
    
    
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch" ,"slam_toolbox", "online_async_launch.py"]
