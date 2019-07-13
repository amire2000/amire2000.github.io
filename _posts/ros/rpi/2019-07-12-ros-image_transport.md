---
layout: post
title: ROS Raspberry PI and image_transport
categories: ros
tags: [rpi, camera, image_transport]
public: 1
description: View and bag camera images from remote machine running ros core
image: ros_rpi.jpeg
---

# Content
- LAB setup
- USB Camera
- Image transport
  - Image transport plugins
  
# LAB
- HW: RPI3+
- OS: Ubuntu 18.04
- ROS: Melodic base
- SW (pc): gnome-system-monitor
- RPI ip: 192.168.2.55
- pc ip: 192.168.2.253



# USB Camera
- Using cv_camera package
  - cv_camera uses OpenCV capture object to capture camera image
  - Publish: image_raw, camera_info

## Install
```
sudo apt install ros-melodic-cv-camera
```

## Run (rpi)
```
rosrun cv_camera cv_camera_node
```

## Topics
```bash
rostopic list
#
cv_camera/camera_info
cv_camera/image_raw
rosout
rosout_agg
```

## image_transport
```bash
rospack list | grep image_tran
image_transport /opt/ros/melodic/share/image_transport
```

> No image_transport_plugins installed, machine doesn' support compressed and theora codec 

&nbsp;  
&nbsp;  
&nbsp;  
#  image transport plugins
[Wiki](http://wiki.ros.org/image_transport_plugins)  
A set of plugins for publishing and subscribing to sensor_msgs/Image topics in representations other than raw pixel data
- compressed_image_transport: plugin providing JPEG and PNG compression of still images
- theora_image_transport: plugin using Theora video codec


### Install
```
sudo apt install ros-melodic-image-transport-plugins 
```

# LAB Run
## PC SW
### Install gnome-system-monitor
```
sudo apt install gnome-system-monitor
```

## Set environment variables
### RPI
```
export ROS_IP=192.168.2.55
export ROS_MASTER_URI=http://192.168.2.55:11311
```

### PC
export ROS_IP=192.168.2.253
export ROS_MASTER_URI=http://192.168.2.55:11311

## RPI
### package structure
```
camera_101
└── launch
     └── cv_cam.launch
```

- cv_cam.launch
```xml
<?xml version="1.0"?>
<launch>
    <node pkg="cv_camera" type="cv_camera_node" name="cv_camera_node" >
        <param name="device_id" value="0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
    </node>
</launch>
```
### Usage
- Run launch file

```
roslaunch camera101 cv_cam.launch
```

## PC
- Run `rqt`
  - Add image visualizer
  - Switch between raw and compress topics
- Run `gnome-system-monitor`
  - view network graph

## Lab results
- RAW Image
  
![](/images/2019-07-13-18-49-19.png)

- Compress Image

![](/images/2019-07-13-18-50-26.png)


# using rosbag on remote machine
```bash
#Run rosbag
rosbag record -O /tmp/rpi /cv_camera_node/image_raw/compressed

```