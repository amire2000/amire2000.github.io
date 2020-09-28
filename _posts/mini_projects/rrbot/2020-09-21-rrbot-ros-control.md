---
layout: post
title: RRbot - ROS control
categories: [mini, rrbot, ros]
tags: [ros, gazebo, urdf]
public: true
description: RRBot and ROS Control
---

# Transmission
Describe the relationship between an actuator and a joint


# yaml config 
- `joint_state_controller`: This controller reads all joint positions and publish them on to the topic "/sensor_msgs/joint_states"
- `effort_controllers`: Command a desired force/torque to joints
  - JointPositionController
  - JointVelocityController
  - JointEffortController

## Physic reminder
### Torque
is a measure of the force that can cause an object to rotate about an axis, units (Nm)
![](/images/2020-09-20-06-55-03.png)
$$
T = F*r*sin(theta)
$$
### Force 
is what cause an object to accelerate 


# RQT
![](/images/2020-09-19-22-13-06.png)

&nbsp;  
# Reference
- [Gazebo ROS control](http://gazebosim.org/tutorials/?tut=ros_control)
- [Blog Robotics ROS Control](https://www.rosroboticslearning.com/ros-control)
- [[ROS Q&A] 112 - How to manually tune a PID with ROS Control](https://youtu.be/gA-O39LrXzI)
- [](https://byjus.com/physics/torque/)