FROM ros:humble

# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get -y install python3-pip && apt-get install -y --no-install-recommends apt-utils

RUN pip3 install opencv-python

COPY ./usb_cam /ros2_usb_camera_ws/src

WORKDIR /ros2_usb_camera_ws

#RUN apt-get install ros-humble-usb-cam -y 

RUN apt-get update && rosdep install --from-paths src --ignore-src -y

RUN . /opt/ros/humble/setup.sh && colcon build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2","run", "usb_cam", "usb_cam_node_exe"]