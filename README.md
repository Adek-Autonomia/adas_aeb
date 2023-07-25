# adas_aeb
ADAS Advanced Emergency Breaking subsystems. Contains a TOF AEB node, which is hardware-dependent, and a point-cloud based AEB node, which can also run in sim, but needs both LIDAR and a stereo camera.

# additional dependency
vision_msgs, can be installed with:
```
 sudo apt install ros-noetic-vision-msgs
```

## Launchfiles
Including in other launchfiles:
```xml
<include file="$(find adas_aeb)/launch/default.launch">
	<arg name="lidar_topic_" default="..." />
	<arg name="camera_cloud_topic_" default="..." />

	<arg name="stop_topic" default="..." />

	<arg name="use_pcl" default="..." />
	<arg name="use_tof" default="..." />
<include>
```



# TODO
- [X] README.md
