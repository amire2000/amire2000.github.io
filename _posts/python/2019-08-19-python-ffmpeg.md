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
- FFMpeg test: stream capture camera to QGC (udp)
- 
```bash
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

## Stream opencv image using ffmpeg pipe
- Using opencv to create and edit the image
  - Send frame with current date and time
  - > numpy array: high, width, channels
- Run ffmpeg as process
- using pipe to send opencv data to ffmpeg

```python
import cv2
import subprocess as sp
import numpy as np
from datetime import datetime

FFMPEG = "/usr/bin/ffmpeg"
HEIGHT = 480
WIDTH = 640
GREEN_COLOR = (0, 255, 0)
TXT_POS = (20, 20)

command = [
    FFMPEG,
    "-f", "rawvideo",
    "-pix_fmt", "bgr24",
    "-s", "{}x{}".format(WIDTH, HEIGHT), 
    "-r", "30",         # 30 fps
    "-i", "-",          # Read from piped stdin
    "-an",              # No audio
    "-c:v", "libx264",
    "-r", "30",
    "-preset", "ultrafast",
    "-pix_fmt", "yuv420p",
    "-f", "rtp",
    "udp://127.0.0.1:5600"
]


if __name__ == "__main__":
    font_face = cv2.FONT_HERSHEY_COMPLEX
    proc = sp.Popen(command, stdin=sp.PIPE)

    while True:
        frame = np.zeros([HEIGHT, WIDTH,3],dtype=np.uint8)
        cv2.putText(frame, str(datetime.now()), TXT_POS, font_face, 1, GREEN_COLOR)
        cv2.imshow("source", frame)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break

        proc.stdin.write(frame)
```
![](/images/2019-08-26-21-21-56.png)

![](/images/2019-08-26-21-17-32.png)


# Reference
- [ffmpeg-python: Python bindings for FFmpeg](https://github.com/kkroening/ffmpeg-python)
- [ffserver mjpeg](https://gist.github.com/peterhellberg/ebfc72147c2009ee720aafe57ce9c141)