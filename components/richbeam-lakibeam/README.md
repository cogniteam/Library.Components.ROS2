# Lakibeam1-Ros2

<img src="./Lakibeam1-ROS2/lakibeam.png" alt="Lakibeam1-ROS2" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros1-richbeam-lakibeam-driver
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>foxy
</b>

# Short description
* richbeam-lakibeam-lidar driver
License: BSD

# Example usage
```
docker run -it --network=host cognimbus/ros1-richbeam-lakibeam-driver:foxy ros2 launch lakibeam1 lakibeam1_scan.launch.py inverted:=false hostip:=0.0.0.0 port:=2368 angle_offset:=0 scanfreq:=30 filter:=3 laser_enable:=true scan_range_start:=45 scan_range_stop:=315
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


