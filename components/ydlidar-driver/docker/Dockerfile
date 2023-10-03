FROM ros:humble

RUN apt update && apt-get update && apt install -y git

WORKDIR /YDLIDAR

# Install SDK
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    mkdir YDLidar-SDK/build && cd YDLidar-SDK/build && cmake .. && make && make install

# Install ROS driver
RUN git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ws/src/ydlidar_ros2_driver && \
    cd ydlidar_ws/src/ydlidar_ros2_driver && git checkout humble && cd .. && \
    . /opt/ros/humble/setup.sh && cd ../ && colcon build --symlink-install

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /YDLIDAR/ydlidar_ws/install/setup.sh && exec $0 $@"]