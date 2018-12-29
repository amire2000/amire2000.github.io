---
layout: post
title: Connect PX4Flow to pixhawk
categories: pixhawk
tags: [pixhawk, opti-flow, px4flow]
---
Optical Flow Sensor Smart Camera V1.3.1 for PX4 Flight Controller With Sonar, an optical flow smart camera. It has a native resolution of 752Ã—480 pixels and calculates optical flow on a 4x binned and cropped area at 250 Hz (bright, outdoors), giving it a very high light sensitivity. Unlike many mouse sensors, it also works indoors and in low outdoor light conditions without the need for an illumination LED at 120 Hz (dark, indoors). It can be freely reprogrammed to do any other basic, efficient low-level computer vision task.
The PX4FLOW (Optical Flow) Sensor is a specialized high resolution downward pointing camera module and a 3-axis gyro that uses the ground texture and visible features to determine aircraft ground velocity. Although the sensor may be supplied with a built-in Maxbotix LZ-EZ4 sonar to measure height, this has not been reliable enough over a range of surfaces in testing, so its readings are not used.

## PX4Flow

- Connect PX4Flow with usb cable
- Run QGroundControl
- Open Settings

![](/images/2018-12-28-14-28-41.png)

## Adjust Lens Focus
- Insert device to calibration mode, from the Parameters menu set `VIDEO_ONLY=1`
![](/images/2018-12-28-14-43-47.png)
- Adjust the image
- Disabled calibration


## Connection
![](/images/2018-12-29-16-54-09.png)