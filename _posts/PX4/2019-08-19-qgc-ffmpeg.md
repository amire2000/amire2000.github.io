---
layout: post
title: FFMpeg QGC udp stream
categories: px4
tags: [stream]
---

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