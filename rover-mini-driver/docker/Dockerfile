FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update && apt-get upgrade -y && apt-get -y install python3-pip && apt-get install -y --no-install-recommends apt-utils && apt-get install -y ros-foxy-joy

# Rover tools
RUN apt install ros-foxy-desktop python3-argcomplete -y

RUN apt-get install python3-pip -y
RUN apt-get install screen -y
RUN apt-get install nano -y
RUN apt install net-tools -y
RUN apt install can-utils -y
RUN apt install git -y
RUN apt install cmake -y
RUN pip3 install console-menu
RUN pip3 install boto3
RUN pip3 install pexpect
RUN pip install six
RUN pip install colcon-common-extensions

# Rosdep Install
RUN apt install python3-rosdep2 -y
RUN rosdep update
RUN apt install python3-colcon-common-extensions -y

# Cloning this repository into your workspace
RUN mkdir -p ~/rover_driver_ws/src
WORKDIR /root/rover_driver_ws/src
RUN git clone https://github.com/RoverRobotics/roverrobotics_ros2.git -b ros2-foxy-driver


# Install shared library
RUN cd ~/ && mkdir library/
WORKDIR /root/library
RUN git clone https://github.com/RoverRobotics/librover
WORKDIR /root/library/librover/
RUN cmake .
RUN make
RUN sudo make install

# Rebuild your workspace
WORKDIR /root/rover_driver_ws
RUN rosdep install -i --from-path src --ignore-src
RUN . /opt/ros/foxy/setup.sh && colcon build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


