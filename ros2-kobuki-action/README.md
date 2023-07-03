# Ros2-Kobuki-Action

<img src="./ros2-kobuki-action/nimbusc.jpg" alt="ros2-kobuki-action" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros2-kobuki-action
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>galactic
</b>

# Short description
* Local ROS2 gateway

# Example usage
```
docker run -it --network=host --privileged -v /opt/nimbus/data/kobuki_actions/resource/:/kobuki_actions_ws/src/kobuki_actions/resource/json/ cognimbus/ros2-kobuki-action:latest ros2 launch kobuki_actions kobuki_actions.launch.py jsonPath:=/kobuki_actions_ws/src/kobuki_actions/resource/json/kobuki_actions.json
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/mobile_base/commands/velocity | Twist


# Required tf
odom--->base_link


# Provided tf
This node does not provide tf


