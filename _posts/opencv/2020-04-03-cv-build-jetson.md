---
layout: post
title: Build OpenCV with cross compiler for jetson 
categories: opencv
tags: [cross, toolchain, arm, nvidia]
public: true
description: Build Cross Compiler environment on Docker and build opencv, For jetson Nano
---

## Cuda and ...
```bash
cuda-repo-l4t-10-0-local-10.0.326_1.0-1_arm64.deb
#cudnn
sudo dpkg -i libcudnn7_7.6.3.28-1+cuda10.0_arm64.deb
sudo dpkg -i libcudnn7-dev_7.6.3.28-1+cuda10.0_arm64.deb
```

## Prepared

```bash
#TODO: check if python2 dev
sudo apt-get install libpython-dev
sudo apt-get install libpython3-dev
```

## Cmake
```bash
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
..
```


## Test
### Python CUDA
```python
import cv2 as cv
import numpy as np
import os



def test_cuda_upload_download():
    npMat = (np.random.random((128, 128, 3)) * 255).astype(np.uint8)
    cuMat = cv.cuda_GpuMat()
    cuMat.upload(npMat)

    print(np.allclose(cuMat.download(), npMat))

def test_cuda_interop():
    npMat = (np.random.random((128, 128, 3)) * 255).astype(np.uint8)
    cuMat = cv.cuda_GpuMat()
    cuMat.upload(npMat)
    print(cuMat.cudaPtr() != 0)
    stream = cv.cuda_Stream()
    print(stream.cudaPtr() != 0)

if __name__ == '__main__':
    if not cv.cuda.getCudaEnabledDeviceCount():
        print("No CUDA-capable device is detected")
        exit()
    test_cuda_upload_download()
```
# Reference
- [cross aarch64](https://github.com/NVIDIA/TensorRT/blob/master/docker/ubuntu-cross-aarch64.Dockerfile)