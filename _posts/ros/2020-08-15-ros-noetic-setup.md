---
layout: post
title: ROS Noetic Ninjemys
categories: ros
tags: []
description: Install ROS Noetic on ubuntu 20.04
image: NoeticNinjemys.png
public: true
---


# Package
## Create
```
cd ~/catkin_ws/src
catkin_create_pkg tutorials std_msgs rospy roscpp
```

## Build
```bash
catkin make
# source
 . ~/catkin_ws/devel/setup.bash
```

catkin error
- [AttributeError: module 'enum' has no attribute 'IntFlag'](https://answers.ros.org/question/347177/attributeerror-module-enum-has-no-attribute-intflag/)
![](images/2020-08-14-11-34-42.png)

- Fix
  - Remove `enum34`
```
pip3 uninstall -y enum34
```

## resdep
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
#
rospack depends1 tutorial
```