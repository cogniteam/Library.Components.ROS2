FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update && apt-get install -y 
RUN apt-get update && apt install python3-pip -y
RUN apt update -y && sudo apt install locales -y && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8
RUN apt install software-properties-common -y
RUN add-apt-repository universe -y
RUN apt update && sudo apt install curl -y && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && sudo apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
RUN apt install --no-install-recommends -y \
  libcunit1-dev
RUN mkdir -p ~/ros2_foxy/src
WORKDIR /root/ros2_foxy
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos src
RUN apt upgrade -y
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"
RUN apt-get install udev -y
RUN apt-get install ros-foxy-joy -y

COPY ./src /rover_driver_ws/src


RUN cd ~/ && mkdir library/
WORKDIR /root/library
RUN git clone https://github.com/RoverRobotics/librover
WORKDIR /root/library/librover/
RUN cmake .
RUN make
RUN sudo make install


WORKDIR /rover_driver_ws
RUN rosdep install -i --from-path src --ignore-src

RUN . /opt/ros/foxy/setup.sh && colcon build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


