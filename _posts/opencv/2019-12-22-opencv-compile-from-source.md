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
    -D CUDNN_LIBRARY=/usr/lib/aarch64-linux-gnu/libcudnn.so.7.6.3 \
    -D WITH_CUDA=ON \
	-D WITH_CUDNN=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D ENABLE_FAST_MATH=1 \
	-D CUDA_FAST_MATH=1 \
	-D CUDA_ARCH_BIN=6.2 \
	-D WITH_CUBLAS=1 \
    ..
```