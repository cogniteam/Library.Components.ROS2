FROM ros:humble


WORKDIR /rplidar_ros2_ws

RUN apt-get update &&  apt install -y git && rm -rf /var/lib/apt/lists/* && rm -rf

RUN git clone https://github.com/babakhani/rplidar_ros2.git

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
	
	
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
