# Ros2-Slamtec-Rplidar-A2

<img src="./ros2-slamtec-rplidar-a2/slamtec-rplidar-a2-driver.jpg" alt="ros2-slamtec-rplidar-a2" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros2-slamtec-rplidar-a2
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>foxy
</b>

# Short description
* Slamtec RPLidar A2 driver in ROS2
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/ros2-slamtec-rplidar-a2 ros2 launch rplidar_ros rplidar.launch.py frame_id:=laser serial_baudrate:=115200
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


