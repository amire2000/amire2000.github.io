---
layout: post
title: Intel realsense gazebo and ROS
categories: ros
tags: [gazebo, ros, realsense, camera]
---

## Install Gazebo plugin
- [Gazebo RealSense Plugin](https://github.com/intel/gazebo-realsense)

- Clone repo
```
mkdir build
cd build
cmake ..
make
sudo make install
```

> Plugin copy to `/usr/lib/x86_64-linux-gnu/gazebo-9/plugins`
> Module copy to `/usr/share/gazebo-9/models`

### Test
- Run gazebo
- Add/Insert `real_sense model`
- View Topic `realsense/rs/stream/<topic>`


# ROS melodic 
- [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
apt install 
- [ROS Wrapper for Intel® RealSense](https://github.com/intel-ros/realsense#step-3-install-intel-realsense-ros-from-sources)
# LAB Environment
- Docker base on `ros:melodic-ros-base`
- Kernel 4.15
- Run as Root (todo: install as user)

## Install RealSense SDK2.0
> Support D400 and SRR300 series (Not R200)

### Add repository
- Install `software-properties-common` for add-apt-repository
```bash
apt install software-properties-common

apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE 
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE

add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
# update
apt install librealsense2-dkms
apt install librealsense2-utils
# To compile ROS Package
apt install librealsense2-dev
```

# ROS Package
> To compile ROS package install `librealsense2-dev`

### Pre install
```
apt install libeigen3-dev

apt install ros-melodic-cv-bridge

apt install ros-melodic-image-transport
ros-melodic-image-tf
ros-melodic-diagnostic-updater
```
### Git
> Check for request tag (Download tag 2.2.3)
https://github.com/intel-ros/realsense/tree/2.2.3


- Clone into catkin workspace `src` folder
- From catkin_ws folder
```
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Reference
- [intel-ros realsense](https://github.com/intel-ros/realsense/issues/386)