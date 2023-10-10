FROM ros:humble

SHELL ["/bin/bash", "-c"]

WORKDIR /nav2_slam_navigation_ws

COPY ./src /nav2_slam_navigation_ws/src

RUN cd src && \
    git clone -b humble https://github.com/ros-planning/navigation2.git && \
    git clone -b humble https://github.com/SteveMacenski/slam_toolbox.git

RUN apt-get update && apt-get upgrade -y && \ 
    apt-get -y install python3-pip && \
    apt-get install -y --no-install-recommends apt-utils && \
    rosdep install -i --from-path src --ignore-src -y

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# ENTRYPOINT ["/bin/bash", "-c", ". /nav2_slam_navigation_ws/install/setup.sh && exec $0 $@"]

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]