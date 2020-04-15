---
layout: post
title: Matlab C++ tutorial
categories: matlab
tags: [mex]
---

# mex
- MEX stands for MATLAB executable
- All Matlab variable are stored as array
  - In c they declare as `mxArray`

# Demo
- From matlab prompt
- crete file `helloworld.cpp`

```cpp
#include <mex.h>
#include <iostream>

using namespace std;

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    cout << "Hello wolrd\n";
}
```

```
>> mex helloworld.cpp
>> helloworld
```

![](/images/2020-04-13-16-44-32.png)


# loadlibrary
> Don't forget to wrapped cpp function with c function

### c header file
```cpp
#ifndef M2C_H
#define M2C_H

void c_version();

#endif
```

### cpp 
```cpp
#include "m2cpp.hpp"
#include <iostream>

using namespace std;

extern "C" void c_version();

void c_version(){
    version();
}

void version(){
    cout << "version 1.0\n";
}
```

### cmake
```bash
cmake_minimum_required(VERSION 3.10)
project(matlab_ext)

set(CMAKE_CXX_STANDARD 11)

# build library
add_library(mym2cpp SHARED
    m2cpp.cpp
)
target_include_directories(mym2cpp PUBLIC ${PROJECT_SOURCE_DIR}/include)
# copy header file 
add_custom_command(
    TARGET mym2cpp POST_BUILD
    COMMAND ${CMAE_COMMAND} cp
            ${PROJECT_SOURCE_DIR}/include/m2c.h
            ${PROJECT_SOURCE_DIR}/build/src/m2cpp/m2c.h
)
# c tester
add_executable(c_test c_tester.c)
target_link_libraries(c_test PRIVATE mym2cpp)
```

### tester (C)
```c
#include "m2c.h"

int main()
{
    c_version();
    return 0;
}
```

```bash
# Copy header file to build folder
# add path
addpath("/home/user/projects/cpp_zmq/build/src/m2cpp/")
#load
loadlibrary("libmym2cpp.so", "m2c.h")
#list api
libfunctions('libmym2cpp')

Functions in library libmym2cpp:

c_version  

#call
calllib('libmym2cpp','c_version')
version 1.0
#unload
unloadlibrary libmym2cpp
```

# Reference
- [MathWorks loadlibrary](https://www.mathworks.com/help/matlab/ref/loadlibrary.html)