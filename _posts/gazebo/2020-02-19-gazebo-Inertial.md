---
layout: post
title: Gazebo Inertial
categories: gazebo
tags: []
public: true
image:
---
The inertia tensor encodes the mass distribution of a body, so it does depend on the mass, but also on where it is located.

# Make function helper

```
<%def name="box_inertia(x,y,z,mass)">
 <inertial>
    <pose>0 0 0 0 0 0</pose>
    <mass>${mass}</mass>
    <inertia>
      <ixx>${0.0833333 * mass * (y*y + z*z)}</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>${0.0833333 * mass * (x*x + z*z)}</iyy>
      <iyz>0.0</iyz>
      <izz>${0.0833333 * mass * (x*x + y*y)}</izz>
    </inertia>
  </inertial>
</%def>
```

# Reference 
- [
Inertial parameters of triangle meshes
](http://gazebosim.org/tutorials?tut=inertia&cat=build_robot)
- [uos_tools](https://github.com/uos/uos_tools/blob/fuerte/uos_common_urdf/common.xacro)
- [ilecture online](http://www.ilectureonline.com)
