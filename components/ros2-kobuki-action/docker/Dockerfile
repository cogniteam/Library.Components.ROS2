FROM ros:galactic

WORKDIR /kobuki_actions_ws

COPY ./src /kobuki_actions_ws/src

RUN apt -y update && apt -y upgrade

RUN apt-get -y install python3-pip
RUN apt-get -y install ros-galactic-tf2 
RUN apt-get -y install ros-galactic-tf-transformations
RUN pip3 install transforms3d --quiet --exists-action i
RUN . /opt/ros/galactic/setup.sh && colcon build --symlink-install
    
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]