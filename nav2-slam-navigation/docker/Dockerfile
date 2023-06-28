FROM ros:foxy

SHELL ["/bin/bash", "-c"]

WORKDIR /nav2_slam_navigation_ws
COPY ./src /nav2_slam_navigation_ws/src


RUN apt-get update && apt-get upgrade -y && \ 
    apt-get -y install python3-pip && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y ros-foxy-gazebo-ros-pkgs && \
    rosdep install -i --from-path src --ignore-src -y

RUN . /opt/ros/foxy/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]