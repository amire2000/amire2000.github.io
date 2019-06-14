---
layout: post
title: Run RVIZ on remote computer
categories: ros
tags: [rviz, remote]
image: rviz.png
description: RVIZ Tips, Cheat and Remote usage
public: true
---

# Remote usage
## Network setup
- Robot ip: 192.168.2.253
- Remote ip: 192.168.2.114

## Robot environment
```
export ROS_IP=192.168.2.253
export ROS_HOSTNAME=192.168.2.253
export ROS_MASTER_URI=http://192.168.2.253:11311
```

## Remote Setup and environment
### Install
-  Install ros base
```bash
sudo apt-get update

...
# Check install instruction
#http://wiki.ros.org/melodic/Installation/Ubuntu
sudo apt install ros-melodic-ros-base
#rviz
sudo apt-get install rviz
```

### Environment
```
export ROS_IP=192.168.2.114
export ROS_HOSTNAME=192.168.2.114
export ROS_MASTER_URI=http://192.168.2.253:11311
```

# Reference
