---
layout: post
title: Gazebo SDF and sensors
categories: gazebo
tags: [gazebo, sdf, sensors]
image: sensor_logo.png
description: Use SDF to create model with sensors 
public: true
---

## Content
- Sensor SDF element
- Add sensor to model 
- Demo
  - [Camera](#camera)
  - [Depth camera](#depth-camera)
  - [Laser](#laser)
  - [Sonar](#sonar)

## Sensor SDF element
[SDF Specification](http://sdformat.org/spec?ver=1.6&elem=sensor#sensor_visualize)  
We Can add sensor element to `link` or `join` as a child.  
> link and joint can contain many sensors

The sensor element contain child element declare at the type attribute, the element describe the sensor behavior

![](/images/2019-05-13-14-16-21.png)

Sensor has a faw element describe common behavior
- update_rate
- always_on: always update at update_rate
- visualize: Sensor visualized in the GUI
- topic: Name of the topic which data is published.
  - > Necessary for visualized
- plugin: Work with sensor data , ROS topics for example
- pose and frame: position in respect to the specified frame

#### Common sensor element 
```xml
<sensor name="camera" type="camera">
      <always_on>1</always_on>
      <visualize>1</visualize>
      <topic>my_camera</topic>
      <update_rate>20.0</update_rate>
      <sonar>
      ...
      </sonar>
</sensor>
```
## Add sensor type
Using SDF specification to add sensor element 
For example `Sonar` sensor spec.  

![](/images/2019-05-13-20-10-22.png)
&nbsp;  
&nbsp;  
&nbsp;  
# Demos
- Camera
- Depth camera
- LIDAR
- Sonar

&nbsp;  
# Camera
Add rgb (we can control the format) camera
Gazebo publish `gazebo.msgs.ImageStamped` message into `image`  topic

## Camera basic properties
- horizontal_fov: field of view
- image
  - width, height in pixels
  - format: default R8G8B8 (other format check the spec.)
- clip: Objects closer or farther are not render
  - nar, far
```xml
<link>
...
      <sensor name='my_camera1' type='camera'>
      <visualize>1</visualize>
      <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
                  <width>320</width>
                  <height>240</height>
            </image>
            <clip>
                  <near>0.1</near>
                  <far>100</far>
            </clip>
      </camera>
      </sensor>
</link>
```

#### Gazebo with topic visualizer (image view)
![](/images/2019-05-13-14-52-18.png)

```bash
gz topic -l | grep camera
/gazebo/default/camera/link/my_camera1/cmd
/gazebo/default/camera/link/my_camera1/image
```

&nbsp;  
&nbsp;  
&nbsp;  
# Depth camera
Depth camera are camera extension

```xml
<sensor name='my_camera' type='depth'>
      <visualize>1</visualize>
      <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
                  <width>320</width>
                  <height>240</height>
            </image>
            <clip>
                  <near>0.1</near>
                  <far>100</far>
            </clip>
            <depth_camera>
                  <output>depths</output>
            </depth_camera>
      </camera>
</sensor>
```

- Camera to depth camera
  - `camera` element stay
  - Change type to `depth`
  - Add element `depth_camera`

### Depth image 
![](/images/2019-05-13-20-43-03.png)


&nbsp;  
&nbsp;  
&nbsp;  

# laser
- Gazebo publish `laserscan_stamped` message

- Sensor scan behavior
  - horizontal and vertical scan
    - min and max angle in radians


```xml
<sensor name='laser' type='ray'>
      <visualize>1</visualize>
      <topic>/my_laser</topic>
      <ray>
            <scan>
                  <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>-1.570796</min_angle>
                        <max_angle>1.570796</max_angle>
                  </horizontal>
            </scan>
            <range>
                  <min>0.10</min>
                  <max>5.0</max>
                  <resolution>0.01</resolution>
            </range>
      </ray>
</sensor>
```
![](/images/2019-05-13-15-51-47.png)
&nbsp;  
&nbsp;  
&nbsp;  
# sonar
Sonar sensor publish into `sonar` topic message type `gazebo.msgs.SonarStamped`

![](/images/2019-05-13-20-00-47.png)

> Using pose element to rotate the beam to right direction
```xml
<link>
...
      <sensor name='sonar' type='sonar'>
            <always_on>1</always_on>
            <visualize>1</visualize>
            <topic>/my_sonar</topic>
            <pose>0 0 0 0 -1.57 0</pose>
            <sonar>
                  <min>0.3</min>
                  <max>5</max>
                  <radius>0.5</radius>
            </sonar>
      </sensor>
</link>
```
&nbsp;  
&nbsp;  
&nbsp;  
## Reference
- [gazebo plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins)