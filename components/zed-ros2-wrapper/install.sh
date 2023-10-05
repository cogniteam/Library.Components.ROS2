echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
apt update && apt upgrade -y && \
    apt-get -y install python3-pip --no-install-recommends apt-utils && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \ 
    export LANG=en_US.UTF-8 && \ 
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update && sudo apt install curl -y && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \ apt install -y ros-foxy-ros-base python3-argcomplete && \
    apt remove -y ros-foxy-image-transport-plugins ros-foxy-compressed-depth-image-transport ros-foxy-compressed-image-transport && \
    rosdep init
rosdep update