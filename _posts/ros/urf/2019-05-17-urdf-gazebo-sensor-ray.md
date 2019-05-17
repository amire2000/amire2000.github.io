---
layout: post
title: URDF Gazebo ray/laser sensor integration
categories: ros
tags: [urdf, gazebo, sensors, ray, lidar]
description: Embedded gazebo sensors and other behavior in urdf file, View ray sensors in gazebo and rviz, Demo python Node to read sensor data
image: LiDAR.png
public: true
---
# Content
- urdf
  - sensor
  - plugin
- Gazebo and rviz
- Topics and messages
- Code Example

# urdf
## sensor
- **ray** with `libgazebo_ros_laser.so` plugin
- **gpu_ray** with `libgazebo_ros_gpu_laser.so` plugin
  
### Sensor urdf
```xml
<gazebo reference="sensor">
    <sensor type="ray" name="rplidar_sensor">
        <pose>0 0 ${height/2} 0 0 0</pose>
        <visualize>1</visualize>
        <update_rate>40</update_rate>
        <ray>
            <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159265</min_angle>
                    <max_angle>3.14159265</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.2</min>
                <max>6.0</max>
                <resolution>0.01</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
        </ray>
        <plugin name="rplidar_controller" filename="libgazebo_ros_laser.so">
            <topicName>scan</topicName>
            <frameName>world</frameName>
        </plugin>
    </sensor>
</gazebo>
```

## plugin
- topicName: topic name to publish `sensor_msgs/LaserScan`
- frameName: reference frame for tf (in our example world)


# Gazebo and rviz

> RVIZ Tip: Changed `LaserScan` `size` to view scan better 


![](/images/2019-05-17-08-56-22.png)

# Topics and messages
- Topic name set by plugin `topicName` element

```bash
rostopic info /scan
Type: sensor_msgs/LaserScan
```



# Code sample
- Read and display laser scan
  - source from erel robotics
  
```python
#!/usr/bin/env python
import rospy

import cv2
import numpy as np
import math

from sensor_msgs.msg import LaserScan

def callback(data):
    frame = np.zeros((500, 500,3), np.uint8)
    angle = data.angle_max
    Vx = 250
    Vy = 250
    for r in data.ranges:
        if r == float ('Inf'):
            r = data.range_max
        x = math.trunc( (r * 10)*math.cos(angle + (-90*3.1416/180)) )
        y = math.trunc( (r * 10)*math.sin(angle + (-90*3.1416/180)) )
        cv2.line(frame,(250, 250),(x+250,y+250),(255,0,0),1)
        angle= angle - data.angle_increment

    cv2.imshow('frame',frame)
    cv2.waitKey(1)

def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)

    rospy.Subscriber("/scan", LaserScan,callback)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()

```
![](/images/2019-05-17-09-10-02.png)


# References
- [laser scan mesage](https://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/)