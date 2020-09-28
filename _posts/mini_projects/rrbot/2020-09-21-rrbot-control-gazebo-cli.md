---
layout: post
title: RRbot - Control Gazebo from cli
categories: [mini, rrbot, ros]
tags: [ros, gazebo, urdf. cli]
public: true
description: RRBot and ROS Control gazebo
---

# spawn model
```
rosrun gazebo_ros spawn_model \
-sdf \
-database coke_can \
-model coke_can \
-y 2
```