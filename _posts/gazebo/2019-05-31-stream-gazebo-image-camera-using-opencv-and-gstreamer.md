---
layout: post
title: Stream gazebo camera sensor output using OpenCV and gstreamer
categories: gazebo
tags: [camera, stream, opencv, gstreamer]
description: Using gazebo sensor plugin to stream the camera image capture to external application using OpenCV and gstreamer
image: gstreamer-opencv.jpg
public: true
---

# Content
- Install OpenCV with gstreamer support
- Write Gazebo sensor plugin with opencv support
- Gazebo GCV (Gstreamer OpenCV) plugin 
- Python Client usage the stream

# OpenCV with gstreamer support
## Install gstreamer
```bash
sudo apt install \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev
```

## install OpenCV Requirements
```
sudo apt install \
    pkg-config \
    zlib1g-dev  libwebp-dev \
    libtbb2 libtbb-dev  \
    libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
    cmake
```

## clone 
- Clone
- Switch to requested tag

```bash
git clone https://github.com/opencv/opencv.git
# View current tag
git describe --tags
```

## OpenCV cmake options
```bash
cd opencv
mkdir build
cd build
#cmake
cmake \
-D CMAKE_BUILD_TYPE=Release \
-D BUILD_PNG=OFF \
-D BUILD_TIFF=OFF \
-D BUILD_TBB=OFF \
-D BUILD_JPEG=ON \
-D BUILD_JASPER=OFF \
-D BUILD_ZLIB=ON \
-D BUILD_EXAMPLES=OFF \
-D BUILD_opencv_java=OFF \
-D BUILD_opencv_python2=ON \
-D BUILD_opencv_python3=ON \
-D ENABLE_NEON=OFF \
-D WITH_OPENCL=OFF \
-D WITH_OPENMP=OFF \
-D WITH_FFMPEG=OFF \
-D WITH_GSTREAMER=ON \
-D WITH_GSTREAMER_0_10=OFF \
-D WITH_CUDA=OFF \
-D WITH_GTK=ON \
-D WITH_VTK=OFF \
-D WITH_TBB=ON \
-D WITH_1394=OFF \
-D WITH_OPENEXR=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_TESTS=OFF ..

##make
make
##make install
sudo make install
##check
python3
>> print (cv2.__version__)
4.1.0-dev
``` 
&nbsp;  
&nbsp;  
&nbsp;  
# Gazebo Sensor plugin
- Implement camera sensor plugin with OpenCV support

### Project struct
```
├── models
    └── cv_camera
        ├── model.config
        └── model.sdf
├── worlds
    └── cv.world
└── plugins
    ├── bin
        └── libcv_plugin.so
    ├── build
    ├── CMakeLists.txt
    ├── cv_plugin.cpp
    └── cv_plugin.h
```

### cv_plugin.h
{% gist 6beeca67d0f52827ddd334f9c08f3383 %}

### cv_plugin.cpp
{% gist fd577d454555b337a9ae93420ce5f160 %}

### Plug CMakeLists.txt
{% gist 90d9f761b64ce5a47bede5e306f75c93 %}

### cv_camera model
{% gist 4127e797b8722d04ed68c3b3cbf81330 %}

### Set gazebo environment variables
```bash
#GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=<path to project>/models:${GAZEBO_MODEL_PATH}
#GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=<path to project>/plugins/bin
```

- Terminal1 (run gazebo)
```
gazebo --verbose cv.world
```
- The plug launch dialog window show camera output
  
![](/images/2019-05-31-14-00-49.png)


&nbsp;  
&nbsp;  
&nbsp;  
# Gazebo OpenCV Gstreamer
## Gstreamer pipeline
- Sender
```
gst-launch-1.0 -v v4l2src \
! video/x-raw,format=YUY2,width=640,height=480 \
! jpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=5000
```

- Reciver
```
gst-launch-1.0 -v udpsrc port=5000 \
! application/x-rtp, media=video, clock-rate=90000, encoding-name=JPEG, payload=26 \
! rtpjpegdepay \
! jpegdec \
! xvimagesink sync=0
```
## Compile OpenCV with Gstreamer support
- OpenCV version 4.1.0
```
```

# References
- [How to open a GStreamer pipeline from OpenCV with VideoWriter](https://stackoverflow.com/questions/46219454/how-to-open-a-gstreamer-pipeline-from-opencv-with-videowriter)