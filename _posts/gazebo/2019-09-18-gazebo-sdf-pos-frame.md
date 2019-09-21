---
layout: post
title: Gazebo SDF pose and frame
categories: gazebo
tags: [sdf, pose, model]
public: true
description: Pose calculation
image: gpose.png
---


## Transform
Poses in Gazebo represent a transform between two frames. They are defined by 6 values:

`<pose> x y z roll pitch yaw </pose>`

- Translation (x y z) is applied before rotation (roll pitch yaw)
- Fixed axis is used for the rotation

## Reference frame
Currently poses are always defined with respect to a specific frame according to the element it belongs to. 

- Model -> World
- Link -> Model
- Visual -> Link
- Collision -> Link
- Joint -> Child link

## demo
- test model with one link (box)
  - model pose is reference to world
  - link refer to model frame

![](/images/2019-09-20-10-57-19.png)

## demo2
- Joint
  - child link

![](/images/2019-09-20-15-26-24.png)
>Gazebo view `Transparent`, view `joint`

joint pose is relative to child center of mass

Change pose in Z axis 

```xml
<joint name="b2c" type="revolute">
    <parent>box</parent>
    <child>cylinder</child>
    <pose>0 0 -0.05 0 0 0</pose>
    <axis>
        <xyz>0 0 1</xyz>
    </axis>
</joint>
```
![](/images/2019-09-20-15-30-54.png)
# References
- [Pose calculation in gazebosim tutorial](http://answers.gazebosim.org/question/9578/pose-calculation-in-gazebosim-tutorial/)
- 