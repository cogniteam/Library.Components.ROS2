# Cogniteam-Text-Detection

<img src="./cogniteam-text-detection/text_detection.png" alt="cogniteam-text-detection" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/cogniteam-text-detection
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>foxy
</b>

# Short description
* Component for text detection

# Example usage
```
docker run -it --network=host cognimbus/cogniteam-text-detection:foxy ros2 launch cogniteam-text-detection cogniteam-text-detection.launch.py model:=src/cogniteam-text-detection/resource/frozen_east_text_detection.pb width:=640 height:=480 thr:=0.5 nms:=0.4 device:=cpu
```

# Subscribers
ROS topic | type
--- | ---
/image_raw | sensor_msgs/Image


# Publishers
ROS topic | type
--- | ---
/image_bbox | sensor_msgs/Image
/image_bbox/compressed | sensor_msgs/CompressedImage


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


