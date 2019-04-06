---
layout: post
title: Gazebo camera
categories: gazebo
tags: [gazebo, camera]
---


# Create a Video with a Camera
## camera sensor save images
```xml
<sensor name="my_camera">
  <camera>
    <image>
      <width>1920</width>
      <height>1080</height>
    </image>
    ...
    <save enabled="true">
      <path>/tmp/camera_save_tutorial</path>
    </save>
    ...
    <update_rate>30</update_rate>
  </camera>
</sensor>
```
- image width and height
- save images enabled and path
- update rate

## Convert images to video
```
ffmpeg -r 30 \
-pattern_type glob \
-i '/tmp/camera_save_tutorial/default_camera_link_my_camera*.jpg' \
-c:v libx264 my_camera.mp4
```
-i : in files
-pattern_type: infile  import sequence wildcard support
-c : codec name
-r : frame rate ()

## model demo
```xml
<model name='camera'>
    <static>true</static>
    <pose>-1 0 2 0 1 0</pose>
    <link name='link'>
    <visual name='visual'>
        <geometry>
        <box>
            <size>0.1 0.1 0.1</size>
        </box>
        </geometry>
    </visual>
    <sensor name='my_camera' type='camera'>
        <camera>
        <save enabled="true">
            <path>/tmp/camera_save_tutorial</path>
        </save>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
            <width>1920</width>
            <height>1080</height>
        </image>
        <clip>
            <near>0.1</near>
            <far>100</far>
        </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
    </sensor>
    </link>
</model>
```
# Reference
- [Create a Video with a Camera](http://gazebosim.org/tutorials?tut=camera_save&cat=sensors)
- [ffmpeg Documentation](https://ffmpeg.org/ffmpeg.html)