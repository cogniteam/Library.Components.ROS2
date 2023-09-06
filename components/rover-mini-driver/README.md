# Rover-Driver

<img src="./rover-driver/rover-mini-driver.jpg" alt="rover-driver" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/rover-mini-driver
* Supported architectures <b>arm64/amd64/unknown/unknown</b>
* ROS version <b>foxy
</b>

# Short description
* Rover driver for communicating with a Rover Robotics Rover mini.

# Example usage
```
docker run -it --network=host --privileged -v /etc/udev/rules.d/:/etc/udev/rules.d/ cognimbus/rover-mini-driver:foxy ros2 launch roverrobotics_driver mini.launch.py
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


