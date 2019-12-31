---
layout: post
title: OpenCV Gstreamer RTSP 
categories: video
tags: [opencv, rtsp, gstreamer]
public: true
description: using gstreamer rtsp to stream opencv data
---

# python gi module
```bash
sudo apt-get install pkg-config libcairo2-dev gcc python3-dev libgirepository1.0-dev

# 
pip install gobject PyGObject
```

# ubuntu rtsp
```
sudo apt install gir1.2-gst-rtsp-server-1.0
```

# Play rtsp stream
```
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/test latency=300 \
! decodebin \
! autovideosink
```
# reference
- [Write opencv frames into gstreamer rtsp server pipeline](https://stackoverflow.com/questions/47396372/write-opencv-frames-into-gstreamer-rtsp-server-pipeline)
- [GStreamer pipeline to show an RTSP stream](https://stackoverflow.com/questions/44160118/gstreamer-pipeline-to-show-an-rtsp-stream)
- [adaptive-streaming
](https://github.com/shortstheory/adaptive-streaming)
