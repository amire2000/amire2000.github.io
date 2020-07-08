---
layout: post
title: OpenCV Gstreamer RTSP 
categories: video
tags: [opencv, rtsp, gstreamer]
public: true
description: using gstreamer rtsp to stream opencv data
---

# LAB
- Install python binding under virtual env.
- Install gstreamer RTSP support
- Code and run python RTSP server base on gstreamer
  - Play with gstreamer 
- Play stream with opencv

# Install
- Create virtual env

```
python3 -m venv venv
pip install --upgrade wheel pip setuptools
source venv/bin/activate
```

- Install dependencies

```
sudo apt install libgirepository1.0-dev
```

- Install gst bindings

```
pip install PyGObject
```

## RTSP support
```
sudo apt install gir1.2-gst-rtsp-server-1.0
```


# RTSP server
- base on [Source](https://github.com/tamaggo/gstreamer-examples)

```python
#!/usr/bin/env python

# gst-launch-1.0 playbin uri=rtsp://localhost:8554/test

import sys
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GLib, Gst, GstRtspServer, GObject

loop = GLib.MainLoop ()
Gst.init(None)

class MyFactory(GstRtspServer.RTSPMediaFactory):
	def __init__(self):
		GstRtspServer.RTSPMediaFactory.__init__(self)

	def do_create_element(self, url):
		s_src = "v4l2src ! video/x-raw,rate=30,width=320,height=240 ! videoconvert ! video/x-raw,format=I420"
		s_h264 = "videoconvert ! vaapiencode_h264 bitrate=1000"
		s_src = "videotestsrc ! video/x-raw,rate=30,width=320,height=240,format=I420"
		s_h264 = "x264enc tune=zerolatency"
		pipeline_str = "( {s_src} ! queue max-size-buffers=1 name=q_enc ! {s_h264} ! rtph264pay name=pay0 pt=96 )".format(**locals())
		if len(sys.argv) > 1:
			pipeline_str = " ".join(sys.argv[1:])
		print(pipeline_str)
		return Gst.parse_launch(pipeline_str)

class GstServer():
	def __init__(self):
		self.server = GstRtspServer.RTSPServer()
		f = MyFactory()
		f.set_shared(True)
		m = self.server.get_mount_points()
		m.add_factory("/test", f)
		self.server.attach(None)

if __name__ == '__main__':
	s = GstServer()
	loop.run()
```

# Test
## Terminal 1
- Run the server code


## Terminal 2

```bash
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/test latency=300 \
! decodebin \
! autovideosink
```

# OpenCV
- basic capture 
  
```python
import cv2

cap = cv2.VideoCapture("rtsp://localhost:8554/test", cv2.CAP_FFMPEG)

while True:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        break

cap.release()
cv2.destroyAllWindows()
```

&nbsp;  
&nbsp;  
&nbsp;  
# reference
- [Write opencv frames into gstreamer rtsp server pipeline](https://stackoverflow.com/questions/47396372/write-opencv-frames-into-gstreamer-rtsp-server-pipeline)
- [GStreamer pipeline to show an RTSP stream](https://stackoverflow.com/questions/44160118/gstreamer-pipeline-to-show-an-rtsp-stream)
- [adaptive-streaming
](https://github.com/shortstheory/adaptive-streaming)
