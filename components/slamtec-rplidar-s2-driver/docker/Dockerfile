FROM ros:humble

COPY ./sllidar_ros2 /sllidar_ros2_ws/src/sllidar_ros2


WORKDIR /sllidar_ros2_ws

RUN apt-get update 


RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
	
	
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
