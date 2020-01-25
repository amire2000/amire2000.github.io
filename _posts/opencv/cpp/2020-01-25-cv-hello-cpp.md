---
layout: post
title: OpenCV C++ Hello
categories: opencv
tags: [cpp]
description: OpenCV simple application build from command line cmake and meson
public: true
image: opencv_cpp.png
---

# Lab
- write basic opencv cpp application
- compile 
  - command line
  - cmake
  - meson


# Project files

```
.
├── main.cpp
├── build
├── CMakeLists.txt
├── meson.build
└── README.md
```

# app (main.cpp)
- Open camera, capture and show frames

```cpp
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap("/dev/video1");

    // Check if camera opened successfully
    if (!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    while (1)
    {

        Mat frame;
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
            break;

        imshow("Frame", frame);
        char c = (char)waitKey(25);
        if (c == 27)
            break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
```

# Compile
## Command line
- using pkg-config

```
g++ m.cpp -o app `pkg-config --cflags --libs opencv`
```

- find request lib's

```
g++ main.cpp -o app \
    -I /usr/include/opencv2 \
    -L /usr/lib \
    -lopencv_core \
    -lopencv_imgproc \
    -lopencv_videoio \
    -lopencv_highgui 
```

## Compile with cmake
- using pkg-config


```cmake
cmake_minimum_required(VERSION 3.1)
project("cv_cpp_cmake")

set(CMAKE_GXX_FLAGS "-Wall -Wextra -std=c++17")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
set(YOUR_EXCUTABLE "cv_cmake")
add_executable(${YOUR_EXCUTABLE} "main.cpp")
target_link_libraries(${YOUR_EXCUTABLE} ${OpenCV_LIBS})
```

## Compile with meson
- using pkg-config

```python
project('cpp_t', 'cpp',
    version : '0.1',
    default_options : ['warning_level=3', 'cpp_std=c++14'])
    
#/usr/lib/x86_64-linux-gnu/pkgconfig/opencv.pc
opencv = dependency('opencv', version : '>=4')
install_dir = meson.source_root() + '/bin'
src = ['main.cpp']

executable('cap_demo', 
    sources : src, 
    dependencies : opencv,
    install: true,
    install_dir: [install_dir])
```

from meson root folder

```bash
meson build
ninja -C build 
# or
ninja -C build install
```