# Tracer-Mobile-Driver

<img src="./tracer-mobile-driver/tracer-mobile-driver.png" alt="tracer-mobile-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/tracer-mobile-driver
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>foxy
</b>

# Short description
* Rover driver 
License:  GPL

# Example usage
```
docker run -it --network=host --privileged cognimbus/tracer-mobile-driver:latest ros2 launch tracer_base tracer_base.launch.py
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


