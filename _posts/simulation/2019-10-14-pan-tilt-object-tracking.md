---
layout: post
title: Pan/tilt object tracking using OpenCV and Gazebo
categories: robotics
tags: [simulation, object-tracking, gazebo, urdf, zmq, protobuf]
image: eye-tracking.png
description: Implement gimbal object tracking pan/tilt movement using gazebo (urdf) opencv and zmq and protobuf as messaging infrastructure between gazebo and user controller (gcs)
public: true
---

## OpenCV

```python

import cv2
import math

WIN_NAME = "test"
WIDTH  = 640
HEIGHT = 480
FOV = 70 #68.5
VFOV = 56

def calc_vfov():
    r_fov = math.radians(FOV)
    r_vfov = 2 * math.atan(math.tan(r_fov/2)*HEIGHT/WIDTH)
    print (math.degrees(r_vfov))

def calc(x, y):
    h_width = WIDTH/2
    h_hight = HEIGHT/2
    calc_vfov()
    ang_x = (x - h_width)/h_width * FOV/2
    ang_y = (y - h_hight)/h_hight * VFOV/2
    ang_y *= -1
    print (ang_x, ang_y)

def click_and_crop(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print (x, y)
        calc(x, y)

cam = cv2.VideoCapture(1)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cv2.namedWindow(WIN_NAME)
cv2.setMouseCallback(WIN_NAME, click_and_crop)

while True:
    ret, frame = cam.read()
    if not ret:
        break
    cv2.imshow(WIN_NAME, frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cam.release()

cv2.destroyAllWindows()
```

# Reference 
- [Pan/tilt face tracking with a Raspberry Pi and OpenCV](https://www.pyimagesearch.com/2019/04/01/pan-tilt-face-tracking-with-a-raspberry-pi-and-opencv/)
- [Maths - Angle between vectors](http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/index.htm)