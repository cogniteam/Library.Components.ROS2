FROM dustynv/ros:foxy-pytorch-l4t-r32.5.0

SHELL ["/bin/bash", "-c"]
RUN mkdir -p ~/ros_deep_learning/src
WORKDIR /ros_deep_learning/src
RUN git clone --depth=1 https://github.com/dusty-nv/ros_deep_learning
WORKDIR /ros_deep_learning
RUN . /opt/ros/foxy/install/setup.bash && colcon build

COPY ./launch /ros_deep_learning/src/ros_deep_learning/launch

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]



