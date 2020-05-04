---
layout: post
title: Joints and PID
categories: gazebo
tags: [pid]
public: true
---

```xml
<?xml version="1.0"?>
<sdf version="1.5">
  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://mass_on_rails</uri>
      <plugin name="gravity_compensation" filename="libGravityCompensationPlugin.so">
        <uri>model://mass_on_rails</uri>
      </plugin>
    </include>

  </world>
</sdf>
```
# Reference
- [Gravity compensation](http://gazebosim.org/tutorials?tut=gravity_compensation&cat=plugins#GravityCompensation)