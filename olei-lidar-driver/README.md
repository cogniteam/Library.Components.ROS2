# Ros2-Olei-3D-Lidar-Driver

<img src="./ros2-olei-3d-lidar-driver/olei3D.jpg" alt="ros2-olei-3d-lidar-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/olei-lidar-driver
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>foxy
</b>

# Short description
* OLEI 3d Lidar driver in ROS2
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/olei-lidar-driver:foxy ros2 launch ros2_ouster ole3dv2_launch.py laser_frame:=laser lidar_ip:=192.168.1.100 computer_ip:=192.168.1.10 lidar_port:=2368 imu_port:=9866
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


# Ros2-Olei-2D-Lidar-Driver

<img src="./ros2-olei-2d-lidar-driver/olei2D.jpg" alt="ros2-olei-2d-lidar-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/olei-lidar-driver
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>foxy
</b>

# Short description
* OLEI 2D Lidar driver in ROS2
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/olei-lidar-driver:foxy ros2 launch ros2_ouster ole2dv2_launch.py laser_frame:=laser lidar_ip:=192.168.1.100 computer_ip:=192.168.1.10 lidar_port:=2368 imu_port:=9866
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


