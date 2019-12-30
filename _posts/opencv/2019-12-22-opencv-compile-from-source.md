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
	-D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \
	-D BUILD_EXAMPLES=OFF \
    -D WITH_GSTREAMER=ON \
    -D BUILD_IPP_IW=OFF \
    -D WITH_IP=OFF \
    -D BUILD_JAVA=OFF \
    -D BUILD_opencv_java_bindings_generator=OFF \
    -D BUILD_opencv_dnn=OFF \
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