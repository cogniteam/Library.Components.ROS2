# Nav2-Slam-Navigation

<img src="./nav2-slam-navigation/nav2-slam-navigation.png" alt="nav2-slam-navigation" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/nav2-slam-navigation
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>foxy
</b>

# Short description
* This package combines the power of Nav2 and SLAM to provide accurate localization, mapping, and autonomous navigation capabilities to your robot. 
License:  GPL

# Example usage
```
docker run -it --network=host cognimbus/nav2-slam-navigation:foxy ros2 launch nav2_bringup nav2_slam_navigation_cogniteam.launch.py
```

# Subscribers
ROS topic | type
--- | ---
/scan | sensor_msgs/LaserScan
/goal_pose | geometry_msgs/PoseStamped
/odom | nav_msgs/Odometry


# Publishers
ROS topic | type
--- | ---
/map | nav_msgs/OccupancyGrid
/local_costmap/costmap | nav_msgs/OccupancyGrid
/plan | nav_msgs/Path
/nimbus_robot_pose | geometry_msgs/PoseStamped
/cmd_vel | geometry_msgs/Twist


# Required tf
odom--->base_link


# Provided tf
map--->base_link


