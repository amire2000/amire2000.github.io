---
layout: post
title: Python FFMpeg 
categories: python
tags: [stream, qgc]
description: Stream opencv using python and ffmpeg
public: true
image: ffmpeg-icon.jpg
---

## FFMpeg QGC udp stream
```
ffmpeg -f v4l2 \
-i /dev/video1 \
-filter:v scale=640:480 \
-filter:v fps=fps=10 \
-pix_fmt yuv420p \
-c:v libx264 \
-preset ultrafast \
-x264-params crf=23 \
-strict experimental \
-f rtp udp://127.0.0.1:5600
```

# Reference
- [ffmpeg-python: Python bindings for FFmpeg](https://github.com/kkroening/ffmpeg-python)