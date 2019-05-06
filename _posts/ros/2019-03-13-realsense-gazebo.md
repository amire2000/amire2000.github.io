---
layout: post
title: Intel realsense gazebo and ROS
categories: ROS
tags: [gazebo, ros, realsense, camera]
image: intel_realsense_logo.png
public: true
---
# Objective
- Simulate intel realsense in gazebo
- ROS & Intel RealSense 

# Gazebo
## Install Gazebo plugin
- [Gazebo RealSense Plugin](https://github.com/intel/gazebo-realsense)

- Clone repo
- Run `make`
- Run `make install` 
    - plugin: `/usr/lib/x86_64-linux-gnu/gazebo-9/plugins`
  - model: `/usr/share/gazebo-9/models/realsense_camera`
```
mkdir build
cd build
cmake ..
make
sudo make install
```

## Test and view
- Run gazebo
- Add/Insert `real_sense model`
- View Topic `realsense/rs/stream/<topic>`

![](/images/2019-05-06-18-44-28.png)

&nbsp;
&nbsp;  
&nbsp;  
&nbsp; 
# Realsense gazebo plugin
[
Intel RealSense R200 Gazebo ROS plugin and model
](https://github.com/SyrianSpock/realsense_gazebo_plugin)

- Clone
- `catkin build`
- Test

```bash 
roslaunch realsense_gazebo_plugin realsense_urdf.launch
# View ros topics
rqt_image_view /r200/camera/color/image_raw
# View 
rqt_image_view /r200/camera/depth/image_raw
```
![](/images/2019-05-06-20-48-30.png)

- Create point cloud from depth image 
```
roslaunch realsense_gazebo_plugin depth_proc.launc
```

![](/images/2019-05-06-21-10-58.png)
&nbsp;
&nbsp;  
&nbsp;  
&nbsp; 

# ROS Wrapper for realsense SDK
> No gazebo plugin


- [Install Intel® RealSense™ SDK 2.0/librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
 
- [ROS Wrapper for Intel® RealSense](https://github.com/intel-ros/realsense#step-3-install-intel-realsense-ros-from-sources)

## Install RealSense SDK2.0
- Install from pkg (kernel 4.15)

### Add repository
> Install `software-properties-common` for add-apt-repository
```bash
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE 
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE

sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
#-u run update
```
### Install libraries
```bash
sudo apt install librealsense2-dkms
sudo apt install librealsense2-utils
# To compile ROS Package
sudo apt install librealsense2-dev
```

# ROS Package
> To compile ROS package install dev package `librealsense2-dev`

### Pre install
```
sudo apt install libeigen3-dev

sudo apt install ros-melodic-cv-bridge

sudo apt install ros-melodic-image-transport 

sudo apt install ros-melodic-diagnostic-updater
```
### Install from source
> Check for request tag (Download tag 2.2.3)
https://github.com/intel-ros/realsense/tree/2.2.3

- Clone
```bash
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout 2.2.3
# View current tag
git describe --tags

```

- catkin build
```bash
catkin build #-DCATKIN_ENABLE_TESTING=False # -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### Usage
```bash
roslaunch realsense2_camera rs_camera.launch
```

## Reference
- [Intel realsense sdk](https://github.com/IntelRealSense/librealsense/tree/master/doc)
- [intel-ros realsense](https://github.com/intel-ros/realsense/issues/386)
- [Intel RealSense Gazebo ROS plugin and model](https://github.com/SyrianSpock/realsense_gazebo_plugin)