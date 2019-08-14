---
layout: post
title: Capture to video
categories: opencv
tags: [python, capture, video]
description: Capture from usb camera and save frames as video, control camera output with openCV capabilities
public: true
image: motion-sensor.png
---

# Check camera capabilities
```bash
v4l2-ctl --list-formats-ext -d /dev/video1
#
Index       : 1
	Type        : Video Capture
	Pixel Format: 'MJPG' (compressed)
	Name        : Motion-JPEG
		Size: Discrete 640x480
			Interval: Discrete 0.033s (30.000 fps)
			Interval: Discrete 0.050s (20.000 fps)
			Interval: Discrete 0.067s (15.000 fps)
			Interval: Discrete 0.100s (10.000 fps)
			Interval: Discrete 0.133s (7.500 fps)
    
    ...

	Size: Discrete 320x240
			Interval: Discrete 0.033s (30.000 fps)
			Interval: Discrete 0.050s (20.000 fps)
			Interval: Discrete 0.067s (15.000 fps)
			Interval: Discrete 0.100s (10.000 fps)
			Interval: Discrete 0.133s (7.500 fps)

```
&nbsp;  
&nbsp;  
&nbsp;  
# lab setup
- create virtualenv 
- install opencv-python

```bash
(venv) pip install opencv-python
```

# python code
{% gist 0f73fc396664ee5aa6dad0614fc3c781 %}

# Tips
## X264 codec not found
Default opencv install with pip dosn't support H264
using ffmpeg to convert input video to H264
```
ffmpeg -i output.avi -vcodec libx264 video2.mp4
```

## View codec and video properties from vlc
Tools -> code  information
![](/images/2019-08-14-20-52-21.png)
&nbsp;  
&nbsp;  
&nbsp;  

# Reference
- [Video Codecs by FOURCC](http://www.fourcc.org/codecs.php)
- [
Saving Operated Video from a webcam using OpenCV
](https://www.geeksforgeeks.org/saving-operated-video-from-a-webcam-using-opencv/)