---
layout: post
title: Using V4L2loopback and Gazebo Camera
categories: gazebo
tags: [v4loopback, camera]
description: Implement Gazebo plugin that dump the image from camera sensor to v4l2loopback for future use.
public: true
image: gazebo.png
---

# v4l2loopback
## install
```
sudo apt install v4l2loopback-dkms
sudo apt install v4l2loopback-utils
```

## usage
```bash
# Create /dev/video3, /dev/video4, /dev/video7
modprobe v4l2loopback video_nr=3,4,7

# Forcing a gstreamer caps
v4l2loopback-ctl set-caps "video/x-raw,format=UYVY,width=640,height=480" /dev/video0
```
&nbsp;  
&nbsp;  
&nbsp;  
# Gazebo
> Don't forget setting `GAZEBO_PLUGIN_PATH`

## Module
{% gist 58f037d09259661891efe3cf960988f6 %}


## Plugin
- header
- Code
- CMakeLists.txt

### header
{% gist 283739cd846b5328e563a243c7606386 %}
### Code
{% gist 7055a9aea7ae14b3982e3f4df4d09b04 %}
### CMakeLists.txt
```cmake
set(PLUG_NAME v4l_camera_plugin)

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
add_library(${PLUG_NAME} SHARED v4l_camera.cpp)
target_link_libraries(${PLUG_NAME} 
    ${GAZEBO_LIBRARIES}
    CameraPlugin
    ${OpenCV_LIBS})
```

### Usage
#### Plugin argument
```xml
<plugin name="cv" filename="libv4l_camera_plugin.so">
    <device>/dev/video2</device>
    <debug>true</debug>
</plugin>
```
- device: path to v4l2loopback device
- debug: true, output fps to gazebo console

#### Get image with gstreamer
```
gst-launch-1.0 v4l2src device=/dev/video2 ! autovideosink
```

![](/images/2020-03-26-16-12-36.png)
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [](https://github.com/umlaeute/v4l2loopback)