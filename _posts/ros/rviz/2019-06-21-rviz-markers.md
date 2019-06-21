---
layout: post
title: RVIZ Markers
categories: ros
tags: [rviz, markers]
image: marker.jpg
description: Shows how to use visualization_msgs/Marker messages to send basic shapes (cube, sphere, cylinder, arrow) to rviz.
public: true
---
# Content
- Create basic marker
- Code explain (python)
- Usage

Unlike other displays, the Marker Display lets you visualize data in rviz without rviz knowing anything about interpreting that data. Instead, primitive objects are sent to the display through `visualization_msgs/Marker` messages, which let you show things like arrows, boxes, spheres and line

# Code
- Create package
  - Send marker
- Code explain

## Create pkg
```bash
catkin_create_pkg my_rviz_markers rospy visualization_msgs
```

## Python marker node
- publish marker message

{% gist b15840fc280bfdbf059689b2401e3191 %}

- Set reference frame id and time stamp for `tf` info

```python
self.marker_obj.header.frame_id = "/odom"
self.marker_obj.header.stamp = rospy.get_rostime()
```

- Marker namespace and id for unique
  
```python
self.marker_obj.ns = "robotX"
self.marker_obj.id = index
```

- Marker shape and Action
  - Add
  - Delete
  
```python
self.marker_obj.type = Marker.SPHERE
self.marker_obj.action = Marker.ADD
```

> Other marker properties see [...](http://wiki.ros.org/rviz/DisplayTypes/Marker#Message_Parameters)
&nbsp;  
&nbsp;  
&nbsp;  

# Usage
- Terminal 1

```bash
roscore
```

- Terminal 2
> Don't forget to add executable permission
```bash
rosrum my_rviz_markers basic_marker.py
```

- Terminal 3
  - Run rviz
  - Set `Fixed Frame` to `odom`
  - Add `Marker`
    - set `Marker Topic` to `marker_basic`
```bash
rosrun rviz rviz
```

![](/images/2019-06-21-07-26-30.png)

&nbsp;  
&nbsp;  
&nbsp;  
# Resource
- [Markers: Sending Basic Shapes (C++)](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)
- [ROS Developers LIVE-Class #24: How to create basic markers in ROS Rviz](https://www.youtube.com/watch?v=5pGzW-M6iGQ)
- [JSK](https://jsk-visualization.readthedocs.io/en/latest/index.html)
- [Python visualization_msgs.msg.Marker() Examples ](https://www.programcreek.com/python/example/88812/visualization_msgs.msg.Marker)
- [Rviz Display types](http://wiki.ros.org/rviz/DisplayTypes/Marker#Message_Parameters)