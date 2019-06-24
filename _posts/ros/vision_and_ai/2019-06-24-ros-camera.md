---
layout: post
title: ROS Camera 
categories: ros
tags: [vision, usb_camera. uvc]
image: camera_icon.png
description: How to work with different ROS camera packages
public: true
---
# Content
- [cv_camera](#cvcamera)
- [libuvc_camera](#libuvccamera)
- [web video server](#web-video-server)

This post show how to work with different ROS camera package and camera manager and utils

# cv_camera
[ROS Wiki](http://wiki.ros.org/cv_camera)
## Install
```
sudo apt install ros-melodic-cv-camera
```

## Usage
- Terminal 1 (roscore)
```
roscore
```

- Terminal 2
```bash
#Camera device id
rosparam set cv_camera/device_id 1
rosrun cv_camera cv_camera_node
```

### using launch file
- Check ros wiki for full parameters

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="cv_camera" type="cv_camera_node" name="cv" >
        <param name="device_id" value="1"/>
        <param name="image_width" value="1280"/>
        <param name="image_height" value="800"/>
    </node>
</launch>
```

### View image using rqt
> Tip: Check original zoom when working with rqt
```
rqt_image_view
```

![](/images/2019-06-25-01-22-58.png)

&nbsp;  
&nbsp;  
&nbsp;  
# libuvc_camera
[ROS Wiki](http://wiki.ros.org/libuvc_camera)

This package provides a ROS interface for digital cameras meeting the USB Video Class standard (UVC) using libuvc. Most webcams are UVC-compliant.

```
sudo apt install ros-melodic-libuvc-camera
```

```
lsusb

Bus 002 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
Bus 002 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 001 Device 004: ID 064e:d213 Suyin Corp. 
Bus 001 Device 003: ID 8087:07da Intel Corp. 
Bus 001 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 003: ID 264a:3006  
Bus 003 Device 029: ID 067b:2303 Prolific Technology, Inc. PL2303 Serial Port
Bus 003 Device 032: ID 045e:0779 Microsoft Corp. LifeCam HD-3000
Bus 003 Device 030: ID 046d:c534 Logitech, Inc. Unifying Receiver

```

```
rosrun libuvc_camera camera_node vendor:=Microsoft Corp. LifeCam HD-3000
[ INFO] [1561400044.824887437]: Opening camera with vendor=0x0, product=0x0, serial="", index=0
[ERROR] [1561400044.825854077]: Permission denied opening /dev/bus/usb/001/004
```

```
sudo chmod o+w /dev/bus/usb/001/004
```

# web video server
[ROS Wiki](http://wiki.ros.org/web_video_server)

## Install
```
sudo apt install ros-melodic-web-video-server
```

## Usage
> roscore are running

- Terminal 1

```bash
# launch one of the camera examples 
rosparam set cv_camera/device_id 1
rosrun cv_camera cv_camera_node
```

- Terminal 2 (web server)

```
rosrun web_video_server web_video_server
[ INFO] [1561415825.581837656]: Waiting For connections on 0.0.0.0:8080
[ INFO] [1561415838.299596634]: Handling Request: /
[ INFO] [1561415838.336175710]: Handling Request: /favicon.ico
[ INFO] [1561415843.612417323]: Handling Request: /stream_viewer?topic=/cv/image_raw
[ INFO] [1561415843.642318049]: Handling Request: /stream?topic=/cv/image_raw
```

- Open browser `localhost:8080`

![](/images/2019-06-25-01-41-54.png)

&nbsp;  
&nbsp;  
&nbsp;  
# image_recorder
[ROS Wiki](http://wiki.ros.org/image_view#image_view.2BAC8-diamondback.video_recorder)
> Remapping image topic to `/image`

> Default file: `output.avi` locate in current path
```
rosrun image_view video_recorder /image:=/cv/image_raw

[ INFO] [1561416818.567201194]: Waiting for topic /cv/image_raw...
[ INFO] [1561416818.821224267]: Starting to record MJPG video at [1280 x 720]@15fps. Press Ctrl+C to stop recording.
^CINFO] [1561416821.651921930]: Recording frame 85

```
