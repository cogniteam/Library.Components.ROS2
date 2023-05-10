FROM saarcogniteam/ros-deep-learning:l4t-r35.1

SHELL ["/bin/bash", "-c"]
WORKDIR /ros_deep_learning

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]




