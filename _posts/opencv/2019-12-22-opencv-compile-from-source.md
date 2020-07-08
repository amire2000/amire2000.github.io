---
layout: post
title: Build OpenCV from source for jetson device
categories: opencv
tags: [source, build, cuda]
description: Build OpenCV With CUDA and DNN backend, OpenCV version 4.3.0
public: true
image: cuda.jpg
---
# LAB
Build OpenCV 4.3 with cuda support

- Script run on jetson nano
- cuda: 10.2
- cudnn: 8.0


## Build
> Clean Previous versions

```bash
# find package to remove
dpkg -l | grep opencv
sudo apt purge libopencv
```
&nbsp;  
&nbsp;  
### Start build script
- Run `install_opencv4.3.0_Jetson.sh` script from [JEP](https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.3.0_Jetson.sh)

Add flags to original script
```bash
# Create `pkg` file as unix-install/opencv4.pc
-D OPENCV_GENERATE_PKGCONFIG=ON \
# Create DEB file to install opencv on other jetson device 
-D CPACK_BINARY_DEB=ON \
# Set DEB file version
-D OPENCV_VCSVERSION=4.3.0
```

```
sudo make install
sudo ldconfig
```
&nbsp;  
&nbsp;  
&nbsp;  
## Create DEB
```bash
# must run make before make package
sudo make package
```
![](/images/2020-07-08-18-41-03.png)


&nbsp;  
&nbsp;  
&nbsp;  
## Python shortcut from virtual env
- Copy / Create link 
  - from `/usr/lib/python3/dist-packages/cv2/` to 
  - to `~/.local/lib/python3.6/site-packages/cv2`
  - or virtualenv folder

```bash
# from venv lib folder create shortcut
ln -s /usr/local/lib/python3.6/dist-packages/cv2/python-3.6/cv2.cpython-36m-aarch64-linux-gnu.so cv2.so
# Run python3
>>> import cv2
>>> cv2.__version__
'4.3.0'
# If other version loaded , check module path with
>>> cv2.__file__
# Remove old .so version and try again
```
&nbsp;  
&nbsp;  
&nbsp;  
# pkg-config
```bash
# Check pkg-config search path
pkg-config --variable pc_path pkg-config
# copy to one of search path
sudo cp unix-install/opencv4.pc /usr/share/pkgconfig
```

## Check
> Don't forget the 4 as a suffix
```
pkg-config --cflags --libs opencv4
```

&nbsp;  
&nbsp;  
&nbsp;  
# Demo
## Simple 
> using `ssh -X` to run gui app 

```cpp
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main (int argc, char* argv[])
{
    cv::Mat img(512, 512, CV_8UC3, cv::Scalar(0));
    cv::putText(img,
        "hello",
        cv::Point2f(100,100),
        cv::FONT_HERSHEY_PLAIN,
        2,
        cv::Scalar(0,0,255,255));
    
    cv::imshow("Result", img);
    cv::waitKey();
    return 0;
}
```

```bash
g++ hello.cpp `pkg-config --cflags --libs opencv4`
```

### using cmake
- CMakeLists.txt
  
```cmake
cmake_minimum_required(VERSION 2.8)
project(hello)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(cv_hello hello.cpp)
target_link_libraries(cv_hello ${OpenCV_LIBS})
```
&nbsp;  
&nbsp;  
&nbsp;  
# CUDA example
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaarithm.hpp"

using namespace cv;

int main (int argc, char* argv[])
{
    try
    {
        cv::Mat src_host = cv::imread("file.png", IMREAD_GRAYSCALE);
        cv::cuda::GpuMat dst, src;
        src.upload(src_host);

        cv::cuda::threshold(src, dst, 128.0, 255.0, THRESH_BINARY);

        cv::Mat result_host;
        dst.download(result_host);
        //cv::imshow("Result", result_host);
        //cv::waitKey();
    }
    catch(const cv::Exception& ex)
    {
        std::cout << "Error: " << ex.what() << std::endl;
    }
    return 0;
}

```

```
g++ hello.cpp `pkg-config --cflags --libs opencv4`
```


# Extract from DEB
- copy `DEB` file to target machine
- run
  
```
sudo dpkg -i *.deb
```

> Im `deb` file missing version run `dpkg` with `-force_bad_version`

# Reference
- [Faster OpenCV for Raspberry Pi](https://github.com/dlime/Faster_OpenCV_4_Raspberry_Pi)