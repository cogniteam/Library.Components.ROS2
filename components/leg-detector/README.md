# Leg-Detector

<img src="./leg-detector/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="leg-detector" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros2-legs-detector
* Supported architectures <b>amd64/arm64/unknown/unknown</b>
* ROS version <b>foxy
</b>

# Short description
* Slam toolbox in ROS2, mapping algorithm
License: LGPL

# Example usage
```
docker run -it --network=host cognimbus/ros2-legs-detector:foxy ros2 launch leg_detector cogniteam_leg_detector.py
```

# Subscribers
ROS topic | type
--- | ---
/scan | sensor_msgs/LaserScan


# Publishers
ROS topic | type
--- | ---
/visualization_marker | visualization_msgs/Marker


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


