---
layout: post
title: ROS USB Camera 
categories: ros
tags: [vision, usb_camera]
image: usb_camera.png
description: Install and calibrate USB camera ros support
public: true
---
# Content
- Install
- Check camera capabilities
- launch file
- [Run](#run)
- [Calibration](#calibration)


# usb_cam
A ROS Driver for V4L USB Cameras  
Interface standard USB cameras and publishes images as `sensor_msgs::Image`

# Installation
```
sudo apt install ros-melodic-usb-cam
```

# Test 123
- check camera support formats, resolution and fps
- run usb_cam_node (`rosrun usb_cam usb_cam_node`) or using launch file
- check topics
- view with rqt_image_view

### list-formats
```bash
v4l2-ctl -d /dev/video0 --list-formats
ioctl: VIDIOC_ENUM_FMT
	Index       : 0
	Type        : Video Capture
	Pixel Format: 'YUYV'
	Name        : YUYV 4:2:2

	Index       : 1
	Type        : Video Capture
	Pixel Format: 'MJPG' (compressed)
	Name        : Motion-JPEG
```

### list-formats-ext
```
v4l2-ctl -d /dev/video0 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
	Index       : 0
	Type        : Video Capture
	Pixel Format: 'YUYV'
	Name        : YUYV 4:2:2
		Size: Discrete 640x480
			Interval: Discrete 0.033s (30.000 fps)
			Interval: Discrete 0.050s (20.000 fps)
			Interval: Discrete 0.067s (15.000 fps)
			Interval: Discrete 0.100s (10.000 fps)
			Interval: Discrete 0.133s (7.500 fps)
```

### Set params in launch file
usb_cam.launch
```xml
<launch>
	<node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
		<param name="video_device" value="/dev/video0" />
		<param name="image_width" value="640" />
		<param name="image_height" value="480" />
		<param name="pixel_format" value="yuyv" />
		<param name="camera_frame_id" value="usb_cam" />
		<param name="io_method" value="mmap"/>
	</node>
</launch>
```

## Run
- Terminal 1
```bash
roscore
```

- Terminal 2 (usb_cam_node)
```
roslaunch vision_tutorial usb_cam.launch
```

- TempTerminal
```
rostopic list
/rosout
/rosout_agg
/usb_cam/camera_info
/usb_cam/image_raw
/usb_cam/image_raw/compressed
```

- Terminal 3 (view)
```
rosrun rqt_image_view rqt_image_view
```

> Note: calibration file not found
> ```
> [ INFO] [1558770996.948585920]: Unable to open camera calibration file [/home/user/.ros/> camera_info/head_camera.yaml]
> [ WARN] [1558770996.948614225]: Camera calibration file /home/user/.ros/camera_info/> head_camera.yaml not found.
> ```

![usb camera image](/images/2019-05-25-01-09-58.png)

&nbsp;  
&nbsp;  
&nbsp;  
# Calibration
- [Download calibration board](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=get&target=check-108.pdf) and print

## Run
- Terminal1
```bash
# Launch 
roslaunch vision_tutorial usb_cam.launch
```

- Terminal2
```
rosrun camera_calibration cameracalibrator.py \
--size 8x6 \
--square 0.108 \
image:=/usb_cam/image_raw camera:=/usb_cam
```

![image calibrate step1](/images/2019-05-25-11-01-20.png)

In order to get a good calibration you will need to move the checkerboard around in the camera frame such that:

- checkerboard on the camera's left, right, top and bottom of field of view
    - X bar - left/right in field of view
    - Y bar - top/bottom in field of view
    - Size bar - toward/away and tilt from the camera 
- checkerboard filling the whole field of view
- checkerboard tilted to the left, right, top and bottom (Skew) 

If the calibrator node gets a sufficient amount of samples, a calibration button will get active on the window. When we press the CALIBRATE button, it will compute the camera
parameters using these samples. It will take some time for calculation. After
computation, two buttons, SAVE and COMMIT, will become active inside the
window, which is shown in the following image. If we press the SAVE button,
it will save the calibration parameters to a file in the /tmp folder. If we press the
COMMIT button, it will save them to ./ros/camera_info/head_camera.yaml

- Restart camera_usb (launch file)
  - Know the node found the camera calibrate yaml file
```
[ INFO] [1558772013.533584540]: using default calibration URL
[ INFO] [1558772013.534549441]: camera calibration URL: file:///home/user/.ros/camera_info/head_camera.yaml
[ INFO] [1558772013.535592561]: Starting 'head_camera' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
```

# Reference
- [usb_cam](http://wiki.ros.org/usb_cam)
- [camera calibration](http://wiki.ros.org/camera_calibration)