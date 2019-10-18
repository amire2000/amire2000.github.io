---
layout: post
title: Install ROS melodic on RPI 3+
categories: ros
tags: [rpi, hw]
image: rpi_ros.jpeg
description: Install ROS Melodic on Raspberry PI 3+ using ubuntu Bionic 18.04
public: true
---
# Content
- Download and flush
- Connect with serial
- Install ssh
- ROS

# Download and flush

Download [Image](http://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.2-preinstalled-server-arm64+raspi3.img.xz) and flash with `Etcher`


# Connect with serial
- Connect pl2302 cable to uart gpio pins
  - 6 (gnd) black
  - 8 tx (white rx)
  - 10 rx (green tx)
- Use putty to connect
  - baud: 115200
  - user: ubuntu
  - pass: ubuntu

![](/images/2019-06-17-20-34-03.png)

![](/images/2019-06-17-20-28-07.png)

## Install ssh

```
sudo apt install openssh-server
```

> Connect with ssh
> Firt try got error `openssh connection reset by port 22 `
> Found that ssh keys fail fail to generated `zero size`
> Regenerated keys with command ` ssh-keygen -f /etc/ssh/ssh_host_rsa_key -N '' -t rsa`

&nbsp;
&nbsp;
&nbsp;
# ROS  Installation
## Pre ROS Install
```
sudo apt install build-essential
```

## Install ROS
[Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

- Setup repo and key and install ros base
- `apt update`
- Install `ros-melodic-ros-base`
- Initialize rosdep and setup environment
- Edit .bashrc



# Reference
- [ubuntu install](http://thomas-messmer.com/index.php/14-free-knowledge/howtos/86-ros-melodic-on-raspberry-pi-3)