---
layout: post
title: V4L loopback install and usage
categories: video
tags: [v4l, camera]
public: True
description: Use loopback to simulation usb/v4l camera
image: loop.png
---

#
- ffmpeg
- v4l2loopback
- v4l-utils

# Install v4l2loopback
```bash
#install
sudo apt instal v4l2loopback-dkms
# load
sudo modprobe v4l2loopback
# check
lsmod | grep v4l

v4l2loopback           40960  0
v4l2_common            16384  1 v4l2loopback

```

## Check for new video device
```bash
v4l2-ctl --list-devices
Dummy video device (0x0000) (platform:v4l2loopback-000):
	/dev/video2

Microsoft® LifeCam HD-3000: Mi (usb-0000:00:14.0-1.2):
	/dev/video1

USB 2.0 UVC HD Webcam: USB 2.0  (usb-0000:00:1a.0-1.3):
	/dev/video0
```

# FFMpeg as stream source
- Download movie (mp4) from [Sample Videos](https://sample-videos.com/)
  
```
ffmpeg -re \
-i  ~/Downloads/SampleVideo_640x360_5mb.mp4 \
-pix_fmt bgr24 \
-map 0:v \
-f v4l2 /dev/video2
```

- `-re` : read input at native frame rate
- `-i` : input file
- `-map 0:v` : map input stream to output
- `-f`: output format
- `/dev/videoX`: checc `v4l2-ctl` for dummy device

> for OpenCV add `-pix_fmts` pixel format  
> for example `-pix_fmt bgr24`  
> Run `ffmpeg -pix_fmts` to get supported format list

## Check with VLC / ffplay
- open capture device with `vlc`

![](/images/2019-08-21-23-16-16.png)

- Run `ffplay /dev/video2`

&nbsp;  
&nbsp;  
&nbsp;  
## Gstreamer example
- Writer
  
```bash
gst-launch-1.0 videotestsrc ! v4l2sink device=/dev/video2
```

- Player

```
ffplay /dev/video2
```
&nbsp;  
&nbsp;  
&nbsp;  
# Python example

using `pyfakewebcam` [github](https://github.com/jremmons/pyfakewebcam) has install instruction end samples
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [v4l2loopback_cpp](https://github.com/eruffaldi/v4l2loopback_cpp)
- [image2 loopback](https://gist.github.com/TimSC/6532334)
- [Stream a mp4 video file to V4l2loopback](https://jiafei427.wordpress.com/2019/04/17/stream-a-mp4-video-file-to-v4l2loopback-device-02/)
- [c write to v4l2loop](https://github.com/umlaeute/v4l2loopback/blob/master/examples/test.c)
- [video_device_from_topic.py](https://gist.github.com/awesomebytes/d51fbd77ab1b887e7c3e)