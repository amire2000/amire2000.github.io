---
layout: post
title: OpenCV tracking ball
categories: OpenCV
tags: [opencv]
---

```python
import numpy as np
import cv2
import os

VIDEO_NAME = "ball_tracking_example.mp4"

greenLower = (29, 86, 6)
greenUpper  =  (64,  255, 255)

video = os.path.join(os.path.dirname(__file__), VIDEO_NAME)

# camera =  cv2.VideoStream(src=0).start()
camera = cv2.VideoCapture(video)


while (True):
    (grabbed, frame) = camera.read()
    h,w = frame.shape[:2]
    image_width = 600
    image_hight = int(h * (image_width/float(h)))    
    dim = (image_hight, image_width)
    resized = cv2.resize(frame, dim)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()
```