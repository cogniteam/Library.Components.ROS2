FROM ros:humble

COPY lakibeam1 /home/lakibeam1_ws

WORKDIR /home/lakibeam1_ws

RUN apt update -y
RUN apt upgrade -y

ENV DEBIAN_FRONTEND=noninteractive
RUN echo "31\n31\n1\n" | apt install -y ros-humble-pcl-conversions

RUN . /opt/ros/humble/setup.sh && colcon build 
