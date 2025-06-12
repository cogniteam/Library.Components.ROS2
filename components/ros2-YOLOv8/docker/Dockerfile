ARG JETPACK_VERSION=5

FROM ultralytics/ultralytics:latest-jetson-jetpack${JETPACK_VERSION}
ARG ROS_EDITION=foxy
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

WORKDIR /yolo_detector_ws
COPY ./src /yolo_detector_ws/src
COPY data.json /yolo_detector_ws/

RUN apt update && \
    apt install -y locales \
    wget && \    
    locale-gen en_US en_US.UTF-8 && \    
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8   && \    
    export LANG=en_US.UTF-8   && \    
    apt install -y software-properties-common   && \    
    add-apt-repository universe   && \    
    apt update && apt install curl -y   && \    
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg   && \    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null   && \    
    apt update && \    
    apt install -y tzdata  && \    
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime  && \    
    dpkg-reconfigure -f noninteractive tzdata  && \
    if [ "$ROS_EDITION" = "foxy" ]; then \
            apt install -y ros-foxy-ros-base python3-argcomplete; \
            # Action A
        elif [ "$ROS_EDITION" = "humble" ]; then \
            apt install -y ros-humble-ros-base; \
            # Action B
        else \
            exit 0 ; \
            # Default action
        fi && \    
    apt install -y python3-colcon-common-extensions   && \    
    apt install -y libboost-all-dev && \
    pip install setuptools==70.0.0 && \
    wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt && \
    wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x.pt

WORKDIR /yolo_detector_ws/src

RUN git clone -b $ROS_EDITION https://github.com/ros-perception/vision_opencv.git

WORKDIR /yolo_detector_ws
RUN . /opt/ros/$ROS_EDITION/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]