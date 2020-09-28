---
layout: post
title: RRbot - urdf
categories: [mini, rrbot, ros]
tags: [ros, gazebo, urdf]
public: true
description: RRBot URDF and Gazebo
---
`URDF` can only specify the kinematic and dynamic properties of a single robot in isolation
`SDF` is a complete description for everything from the world level down to the robot level

RRBot xacro (urdf) include `rrbot.gazebo` for gazebo SDF support
check [URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf)
this file set plugins, colors for gazebo, link fraction and other SDF settings
The urdf link part muts include inertial section



## Test
```
gz sdf -p xxx.urdf
```

