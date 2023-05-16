FROM ros:foxy



COPY ./src /text_detection_ws/src
WORKDIR /text_detection_ws
RUN apt -y update && apt -y upgrade

RUN apt-get -y install python3-pip
RUN rosdep update && rosdep install --from-paths src -y --ignore-src
RUN . /opt/ros/foxy/setup.sh && colcon build
    
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]