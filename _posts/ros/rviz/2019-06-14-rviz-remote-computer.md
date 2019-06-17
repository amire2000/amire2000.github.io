---
layout: post
title: RVIZ Tips
categories: ros
tags: [rviz, remote]
image: rviz.png
description: RVIZ Tips, Cheat and Remote usage
public: true
---
# Content
- Run rviz on remote computer


# Run rviz on remote computer
## Network setup
- Remote: 192.168.2.253
- RPI: 192.168.2.249

## Robot environment
```
export ROS_IP=<RPI>
export ROS_HOSTNAME=<RPI>
export ROS_MASTER_URI=http://<RPI>:11311
```

## Remote Setup and environment
Remote computer has minimum install of ros-base and rviz
> TODO: check with other messages type
### Install
-  Install ros base

```bash
sudo apt-get update
# Check install instruction for ROS base
#http://wiki.ros.org/melodic/Installation/Ubuntu
sudo apt install ros-melodic-ros-base
#rviz
sudo apt-get install rviz
```

### Remote config
- 
```
export ROS_IP=<REMOTE>
export ROS_HOSTNAME=<REMOTE>
export ROS_MASTER_URI=http://<RPI>:11311
```

### Test lab
- View Camera topic from remote robot

- RPI (Robot)
  - Terminal1: `roscore`
  - Terminal2: `rosrun usb_cam usb_cam_node`
- Remote
  - rviz
    - Add image topic 

![](/images/2019-06-17-20-14-19.png)