# Ros2-Generic-Webcam

<img src="./ros2-generic-webcam/generic-webcam-driver.jpg" alt="ros2-generic-webcam" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros2-usb-cam
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>foxy
</b>

# Short description
* Generic webcam driver in ROS2
License: BSD

# Example usage
```
docker run -it --network=host --privileged cognimbus/ros2-usb-cam:latest ros2 run usb_cam usb_cam_node_exe --ros-args -p image_width:=640 -p image_height:=480 -p framerate:=30.0 -p video_device:=/dev/video0 -p pixel_format:=yuyv -p frame_id:=camera
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/image_raw/compressed | sensor_msgs/CompressedImage
/image_raw | sensor_msgs/Image


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


