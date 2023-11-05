FROM stereolabs/zed:3.7-gl-devel-cuda11.4-ubuntu20.04

SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt update && apt upgrade -y && \
    apt-get -y install python3-pip --no-install-recommends apt-utils && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    apt install -y software-properties-common git && \
    add-apt-repository universe && \
    apt update && sudo apt install curl -y

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt install -y ros-foxy-ros-base python3-argcomplete && \
    apt-get install -y ros-foxy-nmea-msgs ros-foxy-geographic-msgs ros-foxy-diagnostic-updater ros-foxy-robot-localization && \
    apt remove -y ros-foxy-image-transport-plugins ros-foxy-compressed-depth-image-transport ros-foxy-compressed-image-transport && \
    apt install ros-dev-tools ros-foxy-xacro && \
    apt install ros-foxy-depthimage-to-laserscan && \
    rosdep init && rosdep update

# Cloning this repository into your workspace
RUN mkdir -p /zed_ros2_wrapper_ws/src
WORKDIR /zed_ros2_wrapper_ws/src
RUN git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --single-branch && \
    git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
    
COPY zed2.launch.py /zed_ros2_wrapper_ws/src/zed-ros2-wrapper/zed_wrapper/launch/zed2.launch.py

WORKDIR /zed_ros2_wrapper_ws/
RUN source /opt/ros/foxy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


