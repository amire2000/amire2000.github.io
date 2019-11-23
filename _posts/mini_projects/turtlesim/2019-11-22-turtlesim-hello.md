---
layout: post
title: Learn ROS with turtlesim - part 01
categories: [mini, turtlesim]
tags: [ros]
image: olympic.png
public: True
description: Learn ROS basic with turtlesim, launch nodes, publish and subscribers and other ROS basics
---

# LAB
- Create package
- Run turtlesim and control it using keyboard teleop
- View topics and messages  
- `remap` topics

# Install
```bash
sudo apt install ros-melodic-teleop-twist-keyboard
sudo apt install ros-melodic-turtlesim
```

# launch
- Run turtlesim node
- Run teleop node
  Map between teleop `publisher` topic to turtle `subscriber`

```xml
<?xml version="1.0"?>
<launch>
    <remap from="/cmd_vel" to="/turtle1/cmd_vel" />
    <node pkg="turtlesim" type="turtlesim_node" name="sim" output="screen"/>
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop" output="screen" launch-prefix="xterm -e" />
</launch>
```

## launch tips
- output node log to console using `output` attribute
- run  node in separate terminal using `launch-prefix` attribute

# remap
mapping between topics
- `teleop_twist_keyboard` node publish `Twist` message in `/cmd_vel` topic
- turtlesim subscribe to `/turtle1/cmd_vel` topic
- launch file redirect / mapping message between topics

### Topic 
- teleop node publish topic `/cmd_vel`
- turtlesim subscribe to `/turtle1/cmd_vel` topic
  
#### before mapping
```bash
rostopic list
/cmd_vel
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

# topic info
## cmd_vel
rostopic info /cmd_vel
Type: geometry_msgs/Twist

Publishers: 
 * /teleop (http://dev:38477/)

Subscribers: None


## turtle cmd_vel
rostopic info /turtle1/cmd_vel
Type: geometry_msgs/Twist

Publishers: None

Subscribers: 
 * /turtlesim (http://dev:39981/)
 * /sim (http://dev:46755/)

```

#### after mapping
```bash
rostopic list
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
#


rostopic info /turtle1/cmd_vel
Type: geometry_msgs/Twist

Publishers: 
 * /teleop (http://dev:45437/)

Subscribers: 
 * /turtlesim (http://dev:39981/)
 * /sim (http://dev:37579/)

```

> `remap` marge between `topic`s
> 
&nbsp;  
&nbsp;  
&nbsp;  
![](/images/2019-11-21-20-58-48.png)

# topics
- publish message into topic using `rostopic pub` command
- subscribe to topic and view message data using `rostopic echo` command

## publish

- `publish` Twist message into `/turtle1/cmd_vel`

### Twist message
[ros.org](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

```bash
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
```

```
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 

```

## subscribe
- subscribe to `/turtle1/pose` topic
- using `teleop` to move

```
rostopic echo /turtle1/pose
```

- `rosmsg show` view message fields
```
rosmsg info turtlesim/Pose
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

![](/images/2019-11-22-00-52-30.png)