---
layout: post
title: Build OpenCV from source
categories: opencv
tags: [source, build]
description:
public: true
image:
---

```
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.1.2.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.1.2.zip

unzip opencv.zip
unzip opencv_contrib.zip

mv opencv-4.1.2 opencv
mv opencv_contrib-4.1.2/ opencv_contrib

```
- List all cmake options

```
cmake -LA
```



```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_ENABLE_NONFREE=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	-D PYTHON_EXECUTABLE=/usr/bin/python3 \
	-D BUILD_EXAMPLES=OFF \
    -D WITH_GSTREAMER=ON \
    -D BUILD_IPP_IW=OFF \
    -D WITH_IP=OFF \
    -D BUILD_JAVA=OFF \
    -D BUILD_opencv_java_bindings_generator=OFF \
    -D BUILD_opencv_dnn=ON \
    -D BUILD_opencv_python2=OFF \
    -D WITH_JASPER=OFF \
    -D WITH_ADE=OFF \
    -D WITH_VTK=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_opencv_xfeatures2d=OFF \
    ..
```

```
make -j8
sudo make install
pkg-config --modversion opencv
```

python3 lib
```
-- Installing: /usr/local/lib/python3.6/dist-packages/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so
-- Set runtime path of "/usr/local/lib/python3.6/dist-packages/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so" to "/usr/local/lib"

```

pkg-config
```
unix-install/opencv4.pc
```

python virtualenv
- from venv `venv/lib/python3.6/site-packages` create softlink

```bash
ln -s /usr/local/lib/python3.6/dist-packages/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so cv2.so
```

# Build Static opencv

## using pkg-config
```bash
export PKG_CONFIG_PATH ~/opencv/bin/pkgconfig/opencv4.pc

pkg-config --cflags opencv4 --libs --static

g++ main.cpp -static -static-libgcc -std=c++11 \
`pkg-config --cflags opencv4 --libs --static`

```

```
cmake -GNinja
ninja
```

# Build With cuda and cudnn for x86
```
cmake -GNinja -DCMAKE_BUILD_TYPE=Release \
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
..
```

```bash
python3 -m sysconfig
# include
python3 -c 'import sysconfig; print(sysconfig.get_paths()["include"])'
# lib
python3 -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])'
# /usr/lib/python3.6/site-packages
#
which python3
#/usr/bin/python3


```

```
update-alternatives: using /usr/include/x86_64-linux-gnu/cudnn_v7.h to provide /usr/include/cudnn.h (libcudnn) in auto mode
```

# Version 4.2
# cuda support

```
 cd ~
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/4.2.0.zip
$ wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.2.0.zip
$ unzip opencv.zip
$ unzip opencv_contrib.zip
$ mv opencv-4.2.0 opencv
$ mv opencv_contrib-4.2.0 opencv_contrib
```

![](/images/2020-03-26-07-25-53.png)

```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_ENABLE_NONFREE=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	-D PYTHON_EXECUTABLE=/usr/bin/python3 \
	-D BUILD_EXAMPLES=OFF \
    -D WITH_GSTREAMER=ON \
    -D BUILD_IPP_IW=OFF \
    -D WITH_IP=OFF \
    -D BUILD_JAVA=OFF \
    -D BUILD_opencv_java_bindings_generator=OFF \
    -D BUILD_opencv_dnn=ON \
    -D BUILD_opencv_python2=OFF \
    -D WITH_JASPER=OFF \
    -D WITH_ADE=OFF \
    -D WITH_VTK=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_opencv_xfeatures2d=OFF \
    -D CUDNN_INCLUDE_DIR=/usr/include/aarch64-linux-gnu \
    -D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so.7.6.5 \
    -D WITH_CUDA=ON \
	-D WITH_CUDNN=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D ENABLE_FAST_MATH=1 \
	-D CUDA_FAST_MATH=1 \
	-D CUDA_ARCH_BIN=6.2 \
	-D WITH_CUBLAS=1 \
    ..
```


```
import time

class EntryExit(object):
    def __init__(self, cb=None):
        pass

    

    def __call__(self, f):
        def wrapper(*arg, **kwargs):
            f(*arg, **kwargs)

        return wrapper

class Obj():
    @staticmethod
    def cb():
        pass

    @EntryExit(cb)
    def func1(self, arg1):
        print(f"{arg1}")

    @EntryExit()
    def func2(self, arg1):
        print(f"{arg1}")

if __name__ == "__main__":
    o = Obj()
    o.func1(1)
    o.func2(2)
```