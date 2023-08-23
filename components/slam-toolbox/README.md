# Ros2-Slam-Toolbox-Mapping

<img src="./ros2-slam-toolbox-mapping/nimbusc.jpeg" alt="ros2-slam-toolbox-mapping" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros2-slam-toolbox
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>eloquent-ros-base-bionic
</b>

# Short description
* Slam toolbox in ROS2, mapping algorithm
License: LGPL

# Example usage
```
docker run -it --network=host cognimbus/ros2-slam-toolbox ros2 launch slam_toolbox online_async_launch.py solver_plugin:=solver_plugins::CeresSolver ceres_linear_solver:=SPARSE_NORMAL_CHOLESKY ceres_preconditioner:=SCHUR_JACOBI ceres_trust_strategy:=LEVENBERG_MARQUARDT ceres_dogleg_type:=TRADITIONAL_DOGLEG ceres_loss_function:=None mode:=mapping odom_frame:=odom map_frame:=map base_frame:=base_link scan_topic:=/scan debug_logging:=false throttle_scans:=1 transform_publish_period:=0.02 map_update_interval:=5 resolution:=0.05 max_laser_range:=20 minimum_time_interval:=0.5 transform_timeout:=0.2 tf_buffer_duration:=30 stack_size_to_use:=40000000 enable_interactive_mode:=true use_scan_matching:=true use_scan_barycenter:=true minimum_travel_distance:=0.5 minimum_travel_heading:=0.5 scan_buffer_size:=10 scan_buffer_maximum_scan_distance:=10 link_match_minimum_response_fine:=0.1 link_scan_maximum_distance:=1.5 loop_search_maximum_distance:=3 do_loop_closing:=true loop_match_minimum_chain_size:=10 loop_match_maximum_variance_coarse:=3 loop_match_minimum_response_coarse:=0.35 loop_match_minimum_response_fine:=0.45 correlation_search_space_dimension:=0.5 correlation_search_space_resolution:=0.01 correlation_search_space_smear_deviation:=0.1 loop_search_space_dimension:=8 loop_search_space_resolution:=0.05 loop_search_space_smear_deviation:=0.03 distance_variance_penalty:=0.5 angle_variance_penalty:=1 fine_search_angle_offset:=0.00349 coarse_search_angle_offset:=0.349 coarse_angle_resolution:=0.0349 minimum_angle_penalty:=0.9 minimum_distance_penalty:=0.5 use_response_expansion:=true
```

# Subscribers
ROS topic | type
--- | ---
/scan | sensor_msgs/LaserScan


# Publishers
ROS topic | type
--- | ---
/map | nav_msgs/OccupancyGrid


# Required tf
odom--->base_link


# Provided tf
map--->base_link
map--->odom


