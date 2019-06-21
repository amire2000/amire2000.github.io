---
layout: post
title: RVIZ Interactive Markers
categories: ros
tags: [rviz, markers]
image: markerII.jpeg
description: Run RVIZ interactive markers to control and send message back from rviz control to your robots
public: true
---
Interactive markers are similar to the "regular" markers, however they allow the user to interact with them by changing their position or rotation, clicking on them or selecting something from a context menu assigned to each marker.

For interactive markers we need to instantiate an `InteractiveMarkerServer` object. This will handle the connection to the client (usually RViz) and make sure that all changes we make are being transmitted and that your application is being notified of all the actions the user performs on the interactive markers. 

![](/images/2019-06-21-18-54-21.png)
> From ROS wiki


```
sudo apt install ros-melodic-jsk-rviz-plugins
```

# Reference
- [Interactive Markers: Getting Started](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started)
- [Interactive Markers: Writing a Simple Interactive Marker Server](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server)