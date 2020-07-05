---
layout: post
title: Build OpenCV from source
categories: opencv
tags: [source, build, cuda]
description: Build OpenCV With CUDA and DNN backend, OpenCV version 4.2.0
public: true
image: cuda.jpg
---
Install OpenCV 4.2 with CUDA DNN backend from source on ubuntu 18.04


- Install NVIDIA Driver (Optinal-If just for build)
- Install CUDA
- Install cudnn
- Install OpenCV pre-requisite
- Download OpenCV source

&nbsp;  
&nbsp;  
&nbsp;  
# opencv 4.2 
- cuda support
- cuda dnn
- python3 support
```
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.2.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.2.0.zip
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-4.2.0 opencv
mv opencv_contrib-4.2.0 opencv_contrib
```

# Build With cuda and cudnn for x86
> Check opencv source code version  
> `cat bin/include/opencv4/opencv2/core/version.hpp`
 
> Check option with `cmake -LA`
```
cmake -DCMAKE_BUILD_TYPE=Release \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
-D INSTALL_TESTS=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D BUILD_EXAMPLES=OFF \
-D BUILD_opencv_world=ON \
-D BUILD_IPP_IW=OFF \
-D WITH_IP=OFF \
-D WITH_1394=OFF \
-D WITH_TIFF=OFF \
-D BUILD_JAVA=OFF \
-D BUILD_opencv_java_bindings_generator=OFF \
-D BUILD_opencv_dnn=ON \
-D BUILD_opencv_python2=OFF \
-D WITH_JASPER=OFF \
-D WITH_ADE=OFF \
-D WITH_VTK=OFF \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_opencv_xfeatures2d=OFF \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda/ \
-D CUDA_FAST_MATH=ON \
-D WITH_CUBLAS=ON \
-D CUDA_ARCH_BIN=5.3,6.0,6.1,7.0,7.5 \
-D CUDA_ARCH_PTX=7.5 \
-D WITH_NVCUVID=ON \
-D WITH_OPENGL=OFF \
-D WITH_MFX=OFF \
-D BUILD_opencv_python2=OFF \
-D BUILD_opencv_python3=ON \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
-D PYTHON_INCLUDE_DIR2=$(python3 -c "from os.path import dirname; from distutils.sysconfig import get_config_h_filename; print(dirname(get_config_h_filename()))") \
-D PYTHON_LIBRARY=$(python3 -c "from distutils.sysconfig import get_config_var;from os.path import dirname,join ; print(join(dirname(get_config_var('LIBPC')),get_config_var('LDLIBRARY')))") \
-D PYTHON3_NUMPY_INCLUDE_DIRS=$(python3 -c "import numpy; print(numpy.get_include())") \
-D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
-D WITH_QT=OFF \
-D WITH_TBB=OFF \
-D WITH_V4L=ON \
-D WITH_EIGEN=ON \
-D OPENCV_SKIP_PYTHON_LOADER=ON \
-D CUDNN_INCLUDE_DIR=/usr/include \
-D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so.7.6.5 \
-D OPENCV_DNN_CUDA=ON \
-D BUILD_opencv_cudacodec=OFF \
-D WITH_OPENCL=OFF \
-D WITH_IMGCODEC_HDR=OFF \
-D WITH_IMGCODEC_SUNRASTER=OFF \
-D WITH_IMGCODEC_PXM=OFF \
-D WITH_IMGCODEC_PFM=OFF \
..
```

### Build and Install
```
make -j8
sudo make install
sudo ldconfig
```


## Fix Python
- Copy / Create link 
  - from `/usr/lib/python3/dist-packages/cv2/` to 
  - to `~/.local/lib/python3.6/site-packages/cv2`
  - or virtualenv folder

```bash
ln -s /usr/lib/python3/dist-packages/cv2.cpython-36m-x86_64-linux-gnu.so cv2.so
# Run python3
>>> import cv2
>>> cv2.__version__
'4.2.0'
# If other version loaded , check module path with
>>> cv2.__file__
# Remove old .so version and try again
```

## Verify installation 
- Run `yolo` on cuda backend

[Sample code of testing functions of OpenCV with CUDA-enabled DNN modules. ](https://github.com/Cuda-Chen/opencv-dnn-cuda-test)

&nbsp;  
&nbsp;  
&nbsp;  
# Build Static opencv

## using pkg-config
```bash
export PKG_CONFIG_PATH ~/opencv/bin/pkgconfig/opencv4.pc

pkg-config --cflags opencv4 --libs --static

g++ main.cpp -static -static-libgcc -std=c++11 \
`pkg-config --cflags opencv4 --libs --static`
```

# Reference
- [Build OpenCV DNN Module with Nvidia GPU Support on Ubuntu 18.04](https://cuda-chen.github.io/image%20processing/programming/2020/02/22/build-opencv-dnn-module-with-nvidia-gpu-support-on-ubuntu-1804.html)


```
cmake \
-DCMAKE_BUILD_TYPE=Release \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
-D INSTALL_TESTS=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D BUILD_EXAMPLES=OFF \
-D BUILD_opencv_world=ON \
-D BUILD_IPP_IW=OFF \
-D WITH_IP=OFF \
-D WITH_1394=OFF \
-D WITH_TIFF=OFF \
-D BUILD_JAVA=OFF \
-D BUILD_opencv_java_bindings_generator=OFF \
-D BUILD_opencv_dnn=ON \
-D BUILD_opencv_python2=OFF \
-D WITH_JASPER=OFF \
-D WITH_ADE=OFF \
-D WITH_VTK=OFF \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_opencv_xfeatures2d=ON \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.0/ \
-D CUDA_FAST_MATH=ON \
-D WITH_CUBLAS=ON \
-D CUDA_ARCH_BIN=6.1 \
-D CUDA_ARCH_PTX=7.5 \
-D WITH_NVCUVID=ON \
-D WITH_OPENGL=OFF \
-D WITH_MFX=OFF \
-D BUILD_opencv_python2=OFF \
-D BUILD_opencv_python3=ON \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
-D PYTHON_INCLUDE_DIR2=$(python3 -c "from os.path import dirname; from distutils.sysconfig import get_config_h_filename; print(dirname(get_config_h_filename()))") \
-D PYTHON_LIBRARY=$(python3 -c "from distutils.sysconfig import get_config_var;from os.path import dirname,join ; print(join(dirname(get_config_var('LIBPC')),get_config_var('LDLIBRARY')))") \
-D PYTHON3_NUMPY_INCLUDE_DIRS=$(python3 -c "import numpy; print(numpy.get_include())") \
-D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
-D WITH_QT=OFF \
-D WITH_TBB=OFF \
-D WITH_V4L=ON \
-D WITH_EIGEN=ON \
-D OPENCV_SKIP_PYTHON_LOADER=ON \
-D CUDNN_INCLUDE_DIR=/usr/include/aarch64-linux-gnu/ \
-D CUDNN_LIBRARY=/usr/lib/aarch64-linux-gnu/libcudnn.so.7.6.3 \
-D WITH_OPENCL=OFF \
-D OPENCV_DNN_OPENCL=OFF \
-D OPENCV_ENABLE_NONFREE=ON \
-D CPACK_BINARY_DEB=ON \
-D WITH_NVCUVID=ON \
-D BUILD_opencv_cudacodec=ON \
..

```

```
pkg-config --variable pc_path pkg-config
```