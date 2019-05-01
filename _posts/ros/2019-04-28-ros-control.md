---
layout: post
title: ROS Controls
categories: ROS
tags: [ros, controls]
---

ROS Control is a set of packages and tools that allow you, basically, to send commands and communicate with the joints of your robot in order to be able to control them.


    effort_controllers
        joint_effort_controller
        joint_position_controller
        joint_velocity_controller
    joint_state_controller
        joint_state_controller
    position_controllers
        joint_position_controller
    velocity_controllers
        joint_velocity_controller

effort_controllers assume the H.W they are controlling support effort input(force, torque) and so produce efforts to implement the specific control 

Depending on the robot joint, we choose the right plugin

## Hardware interface

## Reference
- [How to implement ros_control on a custom robot](https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot)
