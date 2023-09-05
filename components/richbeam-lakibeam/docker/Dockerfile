FROM ros:foxy

COPY lakibeam1 /home/lakibeam1_ws

WORKDIR /home/lakibeam1_ws

RUN apt update -y
RUN apt upgrade -y

ENV DEBIAN_FRONTEND=noninteractive
RUN echo "31\n31\n1\n" | apt install -y ros-foxy-pcl-conversions

RUN . /opt/ros/foxy/setup.sh && colcon build 
