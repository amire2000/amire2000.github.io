---
layout: post
title: ROS Camera 
categories: ros
tags: [vision, usb_camera, uvc, udev]
image: camera_icon.png
description: How to work with different ROS camera packages
public: true
---
# Content
- [cv_camera](#cvcamera)
- [libuvc_camera](#libuvccamera)
- USB Tips
  - [udev rules](#udev)
- Viewers and Savers
  - [web video server](#web-video-server)
  - [Image recorder](#imagerecorder)
  - [Using ROS Bag to capture image](#using-ros-bag)

This post show how to work with different ROS camera package and camera manager and utils to save image and stream data
&nbsp;  
&nbsp;  
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

## Install

```
sudo apt install ros-melodic-libuvc-camera
```

## Find vendor and product for udev rule
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
Or use udevmonitor


```bash
# Disconnect / Connect device
udevadm monitor --subsystem-match=usb --property
## Output
#... part of the output
ID_BUS=usb
ID_MODEL=Microsoft®_LifeCam_HD-3000
ID_MODEL_ENC=Microsoft®\x20LifeCam\x20HD-3000
ID_MODEL_FROM_DATABASE=LifeCam HD-3000
ID_MODEL_ID=0779
ID_REVISION=0106
ID_SERIAL=Microsoft_Microsoft®_LifeCam_HD-3000
ID_USB_INTERFACES=:0e0100:0e0200:010100:010200:
ID_VENDOR=Microsoft
ID_VENDOR_ENC=Microsoft
ID_VENDOR_FROM_DATABASE=Microsoft Corp.
ID_VENDOR_ID=045e
MAJOR=189
MINOR=295
PRODUCT=45e/779/106
SEQNUM=88716

```

- ID_VENDOR_ID=045e
- ID_MODEL_ID=0779

> Tip: http://www.linux-usb.org/usb.ids for vendor and products list

&nbsp;  
&nbsp;  
&nbsp;  
## First run
- Using `rosrun`
- Fail on permission
  
```bash
rosrun libuvc_camera camera_node vendor:=Microsoft Corp. LifeCam HD-3000
# output
[ INFO] [1561400044.824887437]: Opening camera with vendor=0x0, product=0x0, serial="", index=0
[ERROR] [1561400044.825854077]: Permission denied opening /dev/bus/usb/001/004
```

![](/images/2019-07-16-22-29-01.png)

### udev rule
Fix camera permission  with udev rule

- Create udev rule file
- Reload rules
- Disconnect and connect camera to implement the rule
  
&nbsp;  
&nbsp;  

#### Create udev rule file
- Create file `sudo vim /etc/udev/rules.d/99-uvc.rules`
- Paste line
  - > Set camera vendor and product
```
SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="0779", MODE="0666"
```
&nbsp;  
#### Reload rules
```
sudo udevadm control --reload-rules
```
&nbsp;  
#### Reconnect camera
- Disconnect camera from usb port
- Connect again

## launch
```xml
<launch>
  <node pkg="libuvc_camera" type="camera_node" 
    name="libuvc_camera" output="screen">
    <param name="frame_id" value="uvc_camera" />
    <!-- Parameters used to find the camera -->
    <param name="vendor" value="0x045e"/>
    <param name="product" value="0x0779"/>

    <!-- Image size and type -->
    <param name="width" value="640"/>
    <param name="height" value="480"/>
    <param name="video_mode" value="mjpeg"/>
    <param name="frame_rate" value="15"/>
  </node>
</launch>
```

## Usage
```bash
roslaunch camera101 uvc.launch
# Check topics
# View image with rqt
```

&nbsp;  
&nbsp;  
&nbsp;  

# USB Tips
## udev names / symlink
Changed (Add symlink)  device name  with udev `symlink` rule

```
SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="0779", SYMLINK+="MyCamera"
```

- Reload rules `udevadm control --reload`
- Disconnect and Connect device to implement rules
- Check which `ls` command

```bash
ls /dev/MyCam*
#
..... /dev/MyCamera -> bus/usb/003/042
```

&nbsp;  
&nbsp;  
# Viewers and Savers
- web video server
- Image recorder
- ROS Bag

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
[ INFO] [1561416818.821224267]: Starting to record MJPG video at [1280 x 720]@15fps. 
```

&nbsp;  
&nbsp;  
&nbsp;  
# Using ROS Bag
Using ROS bag to recorded and play image data

## Usage
- Terminal 1

```bash
roscore
```

- Terminal 2(camera)

```bash
#Set device id
rosparam set cv_camera/device_id 1
rosrun cv_camera cv_camera_node
```

- Terminal 3 (record)
    - Check topics
    - Run ROS bag


```bash
#Ros topics
rostopic list

/cv_camera/camera_info
/cv_camera/image_raw
/cv_camera/image_raw/compressed
...

#Run rosbag
rosbag record -O subset /cv_camera/image_raw
# -O output

# Hit Ctrl-c

```

---
- Terminal 4 (info and play)

```bash
#bag info
rosbag info subset.bag 
path:        subset.bag
version:     2.0
duration:    17.2s
start:       Jun 25 2019 20:39:49.03 (1561484389.03)
end:         Jun 25 2019 20:40:06.23 (1561484406.23)
size:        454.6 MB
messages:    517
compression: none [517/517 chunks]
types:       sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
topics:      /cv_camera/image_raw   517 msgs    : sensor_msgs/Image

#bag play
rosbag play subset.bag
```

## Bag usage demo
- Save image from bag as video

```bash
# Save video from rosbag
rosrun image_view video_recorder /image:=/cv_camera/image_raw
# Run play
rosbag play subset.bag
```


# Reference
- [ROS Bag Cookbook](http://wiki.ros.org/rosbag/Cookbook)