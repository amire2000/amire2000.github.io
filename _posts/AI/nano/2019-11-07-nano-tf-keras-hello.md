---
layout: post
title: Keras on jetson nano hello
categories: ai
tags: [keras, nano, nvidia, hello]
public: true
description: Install and basic usage Keras library on nvidia nano, 
image: Keras_Logo.jpg
---

```bash
sudo apt install -y git \
    cmake \
    libatlas-base-dev \
    gfortran \
    python3-dev \
    python3-pip \
    libhdf5-serial-dev \
    hdf5-tools
```

```bash
pip install -U pip setuptools --user
pip install --user numpy
pip install --user --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v42 tensorflow-gpu==1.13.1+nv19.3
pip install --user keras
pip install --user jupyter
pip install --user pillow
pip install --user matplotlib
```

```bash
# Pre-requisites
sudo apt-get install build-essential cmake unzip pkg-config
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk-3-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python3-dev
```

```bash
# Download sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.1.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.1.0.zip

# Unzip
unzip opencv.zip
unzip opencv_contrib.zip

# Rename
mv opencv-4.1.0 opencv
mv opencv_contrib-4.1.0 opencv_contrib
```

```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_LIBV4L=ON \
    -D BUILD_opencv_python2=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib/modules \
    -D PYTHON_EXECUTABLE=/usr/bin/python \
    -D BUILD_EXAMPLES=OFF ../opencv
```

![](/images/2019-11-07-21-33-55.png)

```
make -j2
sudo make install
```


# Reference
- [Tutorial: Configure NVIDIA Jetson Nano as an AI Testbed](https://thenewstack.io/tutorial-configure-nvidia-jetson-nano-as-an-ai-testbed/)
- [Line Follower Robot using CNN](https://towardsdatascience.com/line-follower-robot-using-cnn-4bb4f297c672)
- [Keras tutorial](https://elitedatascience.com/keras-tutorial-deep-learning-in-python)
- [Computer Vision Tutorial: A Step-by-Step Introduction to Image Segmentation Techniques (Part 1)](https://www.analyticsvidhya.com/blog/2019/04/introduction-image-segmentation-techniques-python/)

