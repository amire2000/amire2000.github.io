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
sudo modprob v4l2loopback
# check
lsmod | grep v4l

v4l2loopback           40960  0
v4l2_common            16384  1 v4l2loopback

```

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

- Check with VLC
  - open capture device 

![](/images/2019-08-21-23-16-16.png)


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

```python
#!/usr/bin/env python

import fcntl, sys, os
from v4l2 import *
from threading import Thread, Lock
import time
import numpy as np
import scipy.misc as misc
import cv2


def ConvertToYUYV(sizeimage, bytesperline, im):
	padding = 4096
	buff = np.zeros((sizeimage+padding, ), dtype=np.uint8)
	imgrey = im[:,:,0] * 0.299 + im[:,:,1] * 0.587 + im[:,:,2] * 0.114
	Pb = im[:,:,0] * -0.168736 + im[:,:,1] * -0.331264 + im[:,:,2] * 0.5
	Pr = im[:,:,0] * 0.5 + im[:,:,1] * -0.418688 + im[:,:,2] * -0.081312

	for y in range(imgrey.shape[0]):
		#Set lumenance
		cursor = y * bytesperline + padding
		for x in range(imgrey.shape[1]):
			try:
				buff[cursor] = imgrey[y, x]
			except IndexError:
				pass
			cursor += 2
	
		#Set color information for Cb
		cursor = y * bytesperline + padding
		for x in range(0, imgrey.shape[1], 2):
			try:
				buff[cursor+1] = 0.5 * (Pb[y, x] + Pb[y, x+1]) + 128
			except IndexError:
				pass
			cursor += 4

		#Set color information for Cr
		cursor = y * bytesperline + padding
		for x in range(0, imgrey.shape[1], 2):
			try:
				buff[cursor+3] = 0.5 * (Pr[y, x] + Pr[y, x+1]) + 128
			except IndexError:
				pass
			cursor += 4

	return buff.tostring()

class Streamer(Thread):

  _width = 640
  _height = 480
  _frameRate = 30
  _fmt = V4L2_PIX_FMT_YUYV
  _curFrame = None
  _running = False
  _mutex = Lock()

  def __init__(self, devName):
    Thread.__init__(self)
    if not os.path.exists(devName):
      print("Warning: device does not exist",devName)
    self._device = open(devName, 'wr', 0)
    capability = v4l2_capability()
    print("Get capabilities result: %s" % (fcntl.ioctl(self._device, VIDIOC_QUERYCAP, capability)))
    print("Capabilities: %s" % hex(capability.capabilities))
    print("V4l2 driver: %s" % capability.driver)

    self.format = v4l2_format()
    self.format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT
    self.format.fmt.pix.pixelformat = self._fmt
    self.format.fmt.pix.width = self._width
    self.format.fmt.pix.height = self._height
    self.format.fmt.pix.field = V4L2_FIELD_NONE
    self.format.fmt.pix.bytesperline = self._width * 2
    self.format.fmt.pix.sizeimage = self._width * self._height * 2
    self.format.fmt.pix.colorspace = V4L2_COLORSPACE_JPEG

    print("Set format result: %s" % (fcntl.ioctl(self._device, VIDIOC_S_FMT, self.format)))
    self._curFrame = np.zeros((self._height, self._width, 2), dtype=np.uint8)

  def updateFrame(self, frame):
    frame = misc.imresize(frame, (self._height, self._width))
    buff = ConvertToYUYV(
        self.format.fmt.pix.sizeimage,
        self.format.fmt.pix.bytesperline,
        frame)
    self._mutex.acquire()
    self._curFrame = buff
    self._mutex.release()

  def start(self):
    print("Starting Streamer on %s" % self._device)
    self._running = True
    Thread.start(self)

  def stop(self):
    self._running = False
    self.join()

  def run(self):
    while self._running:
      self._mutex.acquire()
      self._device.write(self._curFrame)
      self._mutex.release()
      time.sleep(1./self._frameRate)
```

## usage
- Play images from `test` folder in loop
> Todo: find better\faster `ConvertToYUYV` convert function

```python
import fcntl, sys, os
from v4l2 import *
import time
import scipy.misc as misc
import numpy as np
import cv2
import signal
import sys

import streamer

st = streamer.Streamer('/dev/video2')

def signal_handler(signal, frame):
  print('You pressed Ctrl+C!')
  st.stop()
  sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

imageBuffer = []
for i in range(1,5,1):
  im = misc.imread("test/%s.jpg" %i)
  im = misc.imresize(im, (480, 640))
  imageBuffer.append(im)

st.start()

i = 0
while True:
  st.updateFrame(imageBuffer[i])
  time.sleep(1./5.)
  i+=1
  i%=4

st.stop()
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [v4l2loopback_cpp](https://github.com/eruffaldi/v4l2loopback_cpp)
- [image2 loopback](https://gist.github.com/TimSC/6532334)
- [Stream a mp4 video file to V4l2loopback](https://jiafei427.wordpress.com/2019/04/17/stream-a-mp4-video-file-to-v4l2loopback-device-02/)
- [c write to v4l2loop](https://github.com/umlaeute/v4l2loopback/blob/master/examples/test.c)
- [video_device_from_topic.py](https://gist.github.com/awesomebytes/d51fbd77ab1b887e7c3e)