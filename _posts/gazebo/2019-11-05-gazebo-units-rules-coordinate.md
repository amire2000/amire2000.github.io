---
layout: post
title: Gazebo units and coordinate system
categories: gazebo
tags: [units, coordinate, euler]
public: true

---

# Units
Gazebo uses SI units
- length: meter(m)
- mass: kilogram (Kg)
- time: seconds (s)
- angle: radian (rad)
- force: Newton (N)

# Coordinate frames
Coordinate frame are right-handed
- Z: up
- X: forward
- Y: left
  
![](/images/2019-11-05-07-29-06.png)

Gazebo using `RGB` color to mark AXIS

- R: X axis
- G: Y axis
- B: Z axis

![](/images/2019-11-05-06-27-30.png)

Gazebo using euler angle `rpy`

- X: Roll
- Y: Pith
- Z: Yaw

&nbsp;  
&nbsp;  
# Rules
Euler angles follow the `rpy` order

## Rotation Direction 
Using right hand rule for positive direction

![](/images/2019-11-05-07-04-20.png)
&nbsp;  
&nbsp;  
# Reference frame

- Model -> World
- Link -> Model
- Visual -> Link
- Collision -> Link
- Joint -> Child link
&nbsp;  
&nbsp;  
# Demo
Rotate the box 45 degree along the `Y` axis `Pitch` and in the `Z` axis `Yaw`
> 
```
<link name='box'>
    <pose>0 0 0.1 0 0.707 0.707</pose>
    ....
```
![](/images/2019-11-05-08-10-46.png)
&nbsp;  
&nbsp;  
# Reference
- [Compute roll, pitch and yaw from 3D vector](http://answers.gazebosim.org/question/22154/compute-roll-pitch-and-yaw-from-3d-vector/)
- [Rotations and Orientation](https://www.cs.utexas.edu/~theshark/courses/cs354/lectures/cs354-14.pdf)
- [Allow IMU orientation specification in SDF. Add orientation specification of SphericalCoordinate world frame.
](https://bitbucket.org/osrf/sdformat/pull-requests/284/allow-imu-orientation-specification-in-sdf/diff)
- [Rotation Matrix To Euler Angles](https://www.learnopencv.com/rotation-matrix-to-euler-angles/)