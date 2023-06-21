FROM ros:foxy

COPY ./olelidar_ros2/src /olelidar_ros2_ws/src/


WORKDIR /olelidar_ros2_ws
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y ros-foxy-diagnostic-updater && \
    apt-get install -y python3-colcon-common-extensions && \
    apt-get install -y libpcap-dev && \
    apt-get install ros-foxy-pcl-conversions -y
RUN rosdep install -i --from-path src --ignore-src
RUN . /opt/ros/foxy/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
