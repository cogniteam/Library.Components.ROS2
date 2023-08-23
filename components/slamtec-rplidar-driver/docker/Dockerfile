FROM ros:foxy

COPY ./rplidar_ros2 /rplidar_ros2_ws/src/rplidar_ros2


WORKDIR /rplidar_ros2_ws

RUN apt-get update && rm -rf /var/lib/apt/lists/* && rm -rf /opt/ros/foxy/share/rplidar_ros/launch/rplidar.launch.py


RUN . /opt/ros/foxy/setup.sh && colcon build --symlink-install
	
	
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
