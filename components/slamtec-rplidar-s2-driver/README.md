# Ros2-Slamtec-Rplidar-Driver-S2

<img src="./ros2-slamtec-rplidar-driver-s2/slamtec-rplidar--s2.jpg" alt="ros2-slamtec-rplidar-driver-s2" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/slamtec-rplidar-driver-ros2
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>humble
</b>

# Short description
* Slamtec RPLidar S2 driver in ROS2
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/slamtec-rplidar-driver-ros2:foxy ros2 launch sllidar_ros2 sllidar_s2_launch.py frame_id:=laser serial_baudrate:=1000000
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/scan | sensor_msgs/LaserScan


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


