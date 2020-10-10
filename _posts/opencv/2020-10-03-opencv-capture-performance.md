---
layout: post
title: OpenCV USB camera performance
categories: opencv
tags: [usb, performance, capture, gstreamer]
public: true
description: Compare Capture video from camera method and convert
---

# LAB
- Ubuntu 20.04 (python3)
- i7 - no gpu
- OpenCV: 4.2
- gstreamer: 1.16.2
- Camera: logitech c920
- LAB Working FPS: 20

## Shortcuts
- [Logitech C920](#logitech-c920)
- [GStreamer](#gstreamer)
- [OpenCV](#opdncv)


&nbsp;  
&nbsp;  
## Logitech C920
```bash
v4l2-ctl --list-formats -d /dev/video2
#
ioctl: VIDIOC_ENUM_FMT
	Type: Video Capture

	[0]: 'YUYV' (YUYV 4:2:2)
	[1]: 'H264' (H.264, compressed)
	[2]: 'MJPG' (Motion-JPEG, compressed)

```

### formats , resolution and fps
> View only YUYV 640*480

```
v4l2-ctl --list-formats-ext -d /dev/video2
[0]: 'YUYV' (YUYV 4:2:2)
		Size: Discrete 640x480
			Interval: Discrete 0.033s (30.000 fps)
			Interval: Discrete 0.042s (24.000 fps)
			Interval: Discrete 0.050s (20.000 fps)
			Interval: Discrete 0.067s (15.000 fps)
			Interval: Discrete 0.100s (10.000 fps)
			Interval: Discrete 0.133s (7.500 fps)
			Interval: Discrete 0.200s (5.000 fps)
```

## GStreamer
### RAW
```
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-raw,width=640,height=480,framerate=20/1 \
! videoconvert \
! fpsdisplaysink
```

### MJPEG
```
gst-launch-1.0 v4l2src device=/dev/video2 \
! image/jpeg,width=640,height=480,framerate=20/1 \
! jpegdec \
! videoconvert \
! fpsdisplaysink
```

### H264
```
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-h264,width=640,height=480,framerate=20/1 \
! h264parse \
! avdec_h264 \
! videoconvert \
! fpsdisplaysink
```

### CPU and memory (using htop)

| Format     | CPU(fpsdisplaysink) | cpu(fakesink) |
| ---------- | ------------------- | ------------- |
| RAW (YUYV) | 4.0%                | < 1%          |
| MJPEG      | 10.0%               | 6.5%          |
| H264       | 20.0%               | 11%           |

## APPSINK
- RGB
- GRAY

### RGB/BGR
```
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-raw,width=640,height=480,framerate=20/1 \
! videoconvert \
! video/x-raw,width=640,height=480,framerate=20/1,format=BGR \
! videoconvert \
! appsink
```

### GRAY
```
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-raw,width=640,height=480,framerate=20/1 \
! videoconvert \
! video/x-raw,width=640,height=480,framerate=20/1,format=GRAY8 \
! videoconvert \
! appsink
```

### CPU and memory (using htop)

| Format | CPU(appsink) |
| ------ | ------------ |
| GRAY   | 4.0%         |
| RGB    | 9.0%         |

&nbsp;  
&nbsp;  
# OpenCV
- Read
- Grab and retrieve
- Multithread
- gstreamer pipe

## Read
- Raw
  - RGB: True/False
- MJPEG
  - RGB: True/False

### RAW
```python
import cv2
#tm = cv2.TickMeter()

cap = cv2.VideoCapture("/dev/video2")
cap.set(cv2.CAP_PROP_FPS, 20)
fps = int(cap.get(cv2.CAP_PROP_FPS))
cap.set(cv2.CAP_PROP_CONVERT_RGB, True)
codec = 844715353.0 # YUY2
#codec = 1196444237.0 # MJPG
cap.set(cv2.CAP_PROP_FOURCC, codec)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height =  cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"W:{width}, H:{height},FPS:{fps}")
tm.start()
while True:
    tm.start()
    r, img = cap.read()
    if r == False:
        break
 #   tm.stop()
 #   print(tm.getTimeMilli())
 #   tm.reset()
    cv2.imshow("Video", img)
    k = cv2.waitKey(1)
    
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()
```

### Raw (RGB False)
- Convert to GRAY

```python
cap.set(cv2.CAP_PROP_CONVERT_RGB, True)
#Convert to
cap.set(cv2.CAP_PROP_CONVERT_RGB, False)
```
```python
# Add line
# before imshow
img = cv2.cvtColor(img, cv2.COLOR_YUV2GRAY_YUYV)
```
> Using `img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_UYVY)` result are like RGB=True


### Mjpeg
```python
import cv2

tm = cv2.TickMeter()
cap = cv2.VideoCapture("/dev/video2")
cap.set(cv2.CAP_PROP_FPS, 20)
fps = int(cap.get(cv2.CAP_PROP_FPS))
cap.set(cv2.CAP_PROP_CONVERT_RGB, False)
#codec = 844715353.0 # YUY2
codec = 1196444237.0 # MJPG
cap.set(cv2.CAP_PROP_FOURCC, codec)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height =  cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"W:{width}, H:{height},FPS:{fps}")
tm.start()
while True:
    tm.start()
    r, img = cap.read()
    if r == False:
        break
    img = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)

    tm.stop()
    print(tm.getTimeMilli())
    tm.reset()
    # cv2.imshow("Video", img)
    k = cv2.waitKey(1)
    
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()
```

### CPU and memory (using htop)

| Format          | CPU (imshow) | CPU (Without) |
| --------------- | ------------ | ------------- |
| RGB(true)       | 17.0%        | 10%           |
| RGB(false) gray | 9.0%         | 2.0           |
| mjpg gray       | 11.0%        | 7.0           |


&nbsp;  
&nbsp;  
&nbsp;  

## Gstreamer Pipe
```python
import cv2
tm = cv2.TickMeter()
gst_str = "v4l2src device=/dev/video2 ! video/x-raw,width=640,height=480,framerate=20/1 ! videoconvert ! video/x-raw,width=640,height=480,framerate=20/1,format=GRAY8 ! videoconvert ! appsink"
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
cap.set(cv2.CAP_PROP_CONVERT_RGB, False)

tm.start()

while True:
    tm.start()
    r, img = cap.read()
    if r == False:
        break
    tm.stop()
    print(tm.getTimeMilli())
    tm.reset()

    # cv2.imshow("Video", img)
    k = cv2.waitKey(1)
    
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()
```

### CPU and memory (using htop)

| Format | CPU (imshow) | CPU (Without) |
| ------ | ------------ | ------------- |
| Gray   | 12.0%        | 4.6%          |

# Tips
## using gray source like FLIR
```python
import cv2
cap = cv2.VideoCapture(“/dev/video0”)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
cap.set(cv2.CAP_PROP_CONVERT_RGB, False)
code, frame = cap.read()
```