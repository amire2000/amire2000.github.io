---
layout: post
title: Turtlebot3 SLAM
categories: ros
tags: [gazebo, ros, turtlebot3, slam]
---

# Install
```
sudo apt install ros-melodic-turtlebot3-slam
```
# gmapping
```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
