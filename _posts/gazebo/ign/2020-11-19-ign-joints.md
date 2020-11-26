---
layout: post
title: Gazebo ign joints
categories: gazebo
tags: [ign, joints]
description: ignition gazebo joints
image: ignition_logo_color.svg
public: true
---

![](/images/4032427536-demo_joint_types_2.gif)


# SDF Specification
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="box">

    <link name="link_1">
      ...
    </link>

    <link name="link_2">
      ...
    </link>


    <joint name="bar_12_joint" type="revolute">
      <parent>link_1</parent>
      <child>link_2</child>
      <pose>0 0.5 0 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

  </model>
</sdf>
```

# Tips
- `IGN_GAZEBO_RESOURCE_PATH`: Add modules folder to search path


# Fixed


# Reference
- [demo_joint_types.sdf](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/test/worlds/demo_joint_types.sdf)
- [Dependencies](http://gazebosim.org/tutorials?tut=install_dependencies_from_source)