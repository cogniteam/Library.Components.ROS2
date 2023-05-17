FROM cognimbus/ros-deep-learning:xavier

SHELL ["/bin/bash", "-c"]
WORKDIR /ros_deep_learning

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]