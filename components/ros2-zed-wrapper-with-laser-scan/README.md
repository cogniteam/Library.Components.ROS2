# ZED-Wrapper

<img src="./zed-ros2-wrapper/zed-ros2-wrapper.jpeg" alt="zed-ros2-wrapper" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros2-ZED-wrapper
* Supported architectures <b>arm64</b>
* ROS version <b>foxy</b>

# Short description
* Package that lets you use the ZED stereo cameras with ROS2.

# Example usage
```
docker run -it --gpus all --runtime nvidia --privileged cognimbus/ros2-ZED-wrapper:foxy ros2 launch zed_wrapper zed2.launch.py
```

# Subscribers
ROS topic | type
--- | ---
/cmd_vel | geometry_msgs/Twist


# Publishers
ROS topic | type
--- | ---
/odom_raw | nav_msgs/Odometry


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


