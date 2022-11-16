FROM ros:eloquent-ros-base-bionic


#COPY entrypoint.sh /entrypoint.sh
#RUN chmod +x /entrypoint.sh
#ENTRYPOINT [ "/entrypoint.sh" ]


# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils


RUN mkdir -p colcon_ws/src
RUN cd colcon_ws/src && git clone -b ros2 https://github.com/youngday/rplidar_ros2.git

RUN source /opt/ros/eloquent/setup.sh \
    && cd colcon_ws \
    && rosdep update \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro=eloquent -y
    

WORKDIR colcon_ws/

RUN source /opt/ros/eloquent/setup.bash \
    && colcon build
    
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch" ,"rplidar_ros", "rplidar.launch.py"]
