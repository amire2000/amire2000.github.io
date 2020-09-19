---
layout: post
title: RRbot - hello
categories: [mini, rrbot, ros]
tags: [ros, gazebo, urdf]
public: true
description: Running RRBot from Gazebo_ros_demos package on ROS Noetic 
---

# Download
```bash
cd ~/catkin_ws/src/
git clone https://github.com/ros-simulation/gazebo_ros_demos.git
cd ..
catkin_make
```

# Package
```
├── gazebo_tutorials
│   ├── launch
│   ├── src
│   └── worlds
├── rrbot_control
│   ├── config
│   └── launch
├── rrbot_description
│   ├── launch
│   ├── meshes
│   └── urdf
└── rrbot_gazebo
    ├── launch
    └── worlds

```
&nbsp;  
## Noetic Change 
### rrbot_description
#### rrbot_rviz.launch
- joint_state_publisher
    - Remove `gui` attribute
  
```xml
<node name="joint_state_publisher" 
    pkg="joint_state_publisher_gui" 
    type="joint_state_publisher_gui">
</node>
```

- robot_state_publisher
  - type change from `state_publisher` to `robot_state_publisher`
```xml
<node name="robot_state_publisher" 
    pkg="robot_state_publisher" 
    type="robot_state_publisher"/>
```

### rrbot_control
#### rrbot_control.launch
- robot_state_publisher

 
> Note: install `ros-neotic-ros-control` and `ros-neotic-ros-controllers` packages

# Run
## Rviz
```
roslaunch rrbot_description rrbot_rviz.launch
```

![](/images/2020-09-19-09-09-59.png)


## Gazebo
```
roslaunch rrbot_gazebo rrbot_world.launch
```
![](/images/2020-09-19-09-23-26.png)

## Control
- Terminal1 (gazebo)
    ```
    roslaunch rrbot_gazebo rrbot_world.launch
    ```
- Terminal2 (control)
  - load PID control yaml config file
    ```
    roslaunch rrbot_control rrbot_control.launch
    ```
- Terminal3 (control command)
    ```
    rostopic pub /rrbot/joint1_position_controller/command std_msgs/Float64 "data: 1.5"
    ```

![](/images/2020-09-19-09-32-56.png)
&nbsp;  
&nbsp;  
&nbsp;  
# References
- [Gazebo and ROS Control](https://github.com/JoshMarino/gazebo_and_ros_control)
- [[ROS Q&A] 149 – How to command joint position of a robot in ROS using Python](https://www.theconstructsim.com/ros-qa-149-how-to-command-joint-position-of-a-robot-in-ros-using-python/)