FROM ros:foxy



COPY ./src /text_detection_ws/src
WORKDIR /text_detection_ws
RUN apt -y update && apt -y upgrade

RUN apt-get -y install python3-pip
RUN sudo apt install ros-foxy-cv-bridge -y
RUN apt install ros-foxy-image-transport -y
RUN apt install ros-foxy-vision-msgs -y
RUN . /opt/ros/foxy/setup.sh && colcon build
    
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]