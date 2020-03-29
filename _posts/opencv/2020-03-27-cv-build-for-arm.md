---
layout: post
title: Build OpenCV with cross compiler
categories: opencv
tags: [cross, toolchain, arm]
public: true
description: Build Cross Compiler environment on Docker and build opencv, This demo used RPi with ubuntu OS
---

- Host: PC
  - Ubuntu 18.04
  - Docker base on `ubuntu 18.04`
- Gust: Rpi 3+ 
  - ubuntu 18.04

# Docker
- Build docker base on `ubuntu 18.04`
- Update sources.list
  - support `arm64`
- Install `aarch` cross compiler
- Add arm64 support
- Install amd64 libraries
- Install arm64 dev libraries to build opencv
  - > Tip: add `arm64` architecture support after install amd64 packages
- Add user and ssh support

## Dockerfile
{% gist ca8bfa3bc274c691d672c6a0d567b424 %}

## entrypoint.sh
- Run ssh
- Switch to user

```bash
 #!/bin/bash
set -e
    
    
service ssh start
service ssh status
    
su user
cd /home/user
```

## Sources.list
```
deb http://archive.ubuntu.com/ubuntu/ bionic main restricted
deb http://archive.ubuntu.com/ubuntu/ bionic-updates main restricted
deb http://archive.ubuntu.com/ubuntu/ bionic universe
deb http://archive.ubuntu.com/ubuntu/ bionic-updates universe
deb http://archive.ubuntu.com/ubuntu/ bionic multiverse
deb http://archive.ubuntu.com/ubuntu/ bionic-updates multiverse
deb http://archive.ubuntu.com/ubuntu/ bionic-backports main restricted universe multiverse
deb http://security.ubuntu.com/ubuntu/ bionic-security main restricted
deb http://security.ubuntu.com/ubuntu/ bionic-security universe
deb http://security.ubuntu.com/ubuntu/ bionic-security multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ bionic main restricted universe multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ bionic-updates main restricted universe multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ bionic-backports main restricted universe multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ bionic-security main restricted universe multiverse
```

# Build Docker
```
├── build.sh
├── Dockerfile
├── entrypoint.sh
├── README.md
└── sources.list
```

```
docker build -t cross .
```

# Run
```
docker run -it --rm \
 -v ~/user:/home/user \
 -p 2222:22 \
 --name cross \
 cross:latest
```

## cmake
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/home/user/opencv-4.2.0 \
    -D CMAKE_TOOLCHAIN_FILE=../platforms/linux/aarch64-gnu.toolchain.cmake \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D OPENCV_ENABLE_NONFREE=OFF \
    -D ENABLE_NEON=OFF \
    -D ENABLE_VFPV3=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_DOCS=OFF \
    -D PYTHON3_INCLUDE_PATH=/usr/include/python3.6m \
    -D PYTHON3_LIBRARIES=/usr/lib/arm-linux-gnueabihf/libpython3.6m.so \
    -D PYTHON3_NUMPY_INCLUDE_DIRS=/usr/lib/python3/dist-packages/numpy/core/include \
    -D BUILD_OPENCV_PYTHON2=OFF \
    -D BUILD_OPENCV_PYTHON3=ON \
    -D BUILD_EXAMPLES=OFF ..
```

### Notes
- CMAKE_INSTALL_PREFIX
- CMAKE_TOOLCHAIN_FILE
- PYTHON3_INCLUDE_PATH
- PYTHON3_LIBRARIES
- PYTHON3_NUMPY_INCLUDE_DIRS


## rename python library name
- cp for backup

```
<make install path>/lib/python3.7/dist-packages/cv2/python-3.7/
cp cv2.cpython-37m-x86_64-linux-gnu.so cv2.so
```

&nbsp;  
&nbsp;  
&nbsp;  
# Setup ARM machine
- Install dev packages for opencv support
- Install python3 packages (numpy)
- Copy files from cross machine
- Install opencv into `/opt` folder
- Setup `LD_LIBRARY_PATH`
- Create link to python binding
- Create opencv.pc file 

## Packages
```bash
sudo apt install python3-pip
sudo apt install libgtk-3-dev libcanberra-gtk3-dev
#tiff?
sudo apt install libtiff-dev zlib1g-dev
sudo apt install libjpeg-dev libpng-dev
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
```

## Python packages
```
pip3 install Cython
pip3 install numpy
```

## Copy files from cross
- tar on cross
- scp
- untar
- mv to `/opt`

### Cross machine
# Tar
```
tar -cjvf ~/opencv-4.2.0.tar.bz2 opencv-4.2.0
scp
```
### Target machine
```bash
# untar

# cp to opt

# update opencv.pc
#/usr/lib/aarch64-linux-gnu/pkgconfig/opencv.pc


```

## Setup
### LD
```bash
# update .bashrc
echo 'export LD_LIBRARY_PATH=/opt/opencv-4.2.0/lib:$LD_LIBRARY_PATH' >> .bashrc
# reload
source .bashrc
```
### Python
- create link 
  
```bash
sudo ln -s /opt/opencv-4.2.0/lib/python3.6/dist-packages/cv2 /usr/lib/python3/dist-packages/cv2
```

# opencv.pc
```
libdir = /opt/opencv-4.2.0/lib
includedir = /opt/opencv-4.2.0/include/opencv4

Name: OpenCV
Description: OpenCV (Open Source Computer Vision Library) is an open source computer vision and machine learning software library.
Version: 4.2.0
Libs: -L${libdir} -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_calib3d -lopencv_ccalib -lopencv_core -lopencv_datasets -lopencv_dnn_objdetect -lopencv_dnn -lopencv_dpm -lopencv_face -lopencv_features2d -lopencv_flann -lopencv_freetype -lopencv_fuzzy -lopencv_gapi -lopencv_hfs -lopencv_highgui -lopencv_imgcodecs -lopencv_img_hash -lopencv_imgproc -lopencv_line_descriptor -lopencv_ml -lopencv_objdetect -lopencv_optflow -lopencv_phase_unwrapping -lopencv_photo -lopencv_plot -lopencv_quality -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_shape -lopencv_stereo -lopencv_stitching -lopencv_structured_light -lopencv_superres -lopencv_surface_matching -lopencv_text -lopencv_tracking -lopencv_videoio -lopencv_video -lopencv_videostab -lopencv_xfeatures2d -lopencv_ximgproc -lopencv_xobjdetect -lopencv_xphoto
Cflags: -I${includedir}
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Cross compiling OpenCV 4 for Raspberry Pi and BeagleBone Black ](https://solarianprogrammer.com/2018/12/18/cross-compile-opencv-raspberry-pi-raspbian/)