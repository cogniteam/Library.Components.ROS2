FROM ros:galactic

RUN  apt-get update && apt-get install ros-galactic-cv-bridge ros-galactic-librealsense2 -y && \
    apt-get install ros-galactic-message-filters ros-galactic-image-transport -y && \
    apt-get install ros-galactic-realsense2-camera -y && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev && \
    apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
RUN . /opt/ros/galactic/setup.sh

CMD ["ros2", "launch", "realsense2_camera", "rs_launch.py"]
