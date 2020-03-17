---
layout: post
title: Modern CMake
categories: cpp
tags: [build, cmake]
public: true
description: Using CMake to build project
image: cmake.png
---

# cmake, ccmake, cmake-gui
```
cmake-gui ..
cmake --buid .
```
&nbsp;  
&nbsp;  
&nbsp;  
# Project
```
├── build
├── CMakeLists.txt
├── lib
│   └── mylib
│       ├── CMakeLists.txt
│       ├── example.cpp
│       └── example.hpp
└── src
    ├── CMakeLists.txt
    └── main.cpp
```
&nbsp;  
&nbsp;  
# Files
## root CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.12)
project(hello VERSION 0.1.0)
add_subdirectory(lib/mylib)
add_subdirectory(src)
```

## libs (mylib) CMakeLists.txt
```cmake
# Build lib as STATIC or SHARED(.so) lib
# Default STATIC
# Changed default with BUILD_SHARED_LIBS=TRUE
# set(BUILD_SHARED_LIBS true)
add_library(mylib STATIC example.cpp)

# Set includes used by application need this lib
# CMAKE_CURRENT_SOURCE_DIR
# - Directory contain the current `CMakeLists.txt`
target_include_directories(
    mylib
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Demo show the usage of PUBLIC, PRIVATE and INTERFACE
target_compile_definitions(
    mylib
    PUBLIC
    HELLO_VERSION=4
)
```

## App CMakeLists.txt
```
add_executable(main main.cpp)
target_link_libraries(main PRIVATE mylib)
```

&nbsp;  
&nbsp;  
&nbsp;  
# PRIVATE, PUBLIC, INTERFACE
```
target_compile_definitions(
    mylib
    PUBLIC
    HELLO_VERSION=4
```
- PUBLIC: Both lib and main accessible to `HELLO_VERSION` pre processor variable
- PRIVATE: Only code in `lib` accessible to the variable
- INTERFACE: Only code in `main` or any code that make usage in the library can accessible that variable

## main.cpp
```cpp
#include <iostream>
#include "example.hpp"

using namespace std;

int main()
{
    cout << example::version() << endl;
    cout << "show lib version from main:" << HELLO_VERSION << endl;
    return 0;
}
```

## mylib.example.hpp
```cpp
#ifndef EXAMPLE_H
#define EXAMPLE_H
#include <string>

using std::string;

namespace example{
    string version();
}
#endif
```

## mylib.example.cpp
```cpp
#include <sstream>
#include <string>
#include <iostream>
#include "example.hpp"

using namespace std;

string example::version(){
    ostringstream os;
    os << "version from main:" << HELLO_VERSION << "\n";
    return os.str();
}
```

&nbsp;  
&nbsp;  
&nbsp;  
# More targets
- target_compile_definitions
- target_compile_features
- target_compile_options
- target_include_directories
- target_link_libraries
- target_sources
- get_target_property
- set_target_property
&nbsp;  
&nbsp;  
&nbsp;  
# CMake Scripts

## `Every thing is a string`

### message
- message mode
  - `STATUS`
- > quote output for better formatting

### variable
```bash
set(<variable name> <variable value>)
#usage
message (STATUS ${<varriable name>})
```

### if
```cmake
if ()
elseif()
else()
endif()
```

- STREQUAL: string 
- EQUAL: numbers
- MATCHES: regex
```cmake
set (my_bool TRUE)
if (my_bool)
    message(STATUS "run met condition")
else()
    message(STATUS "run un met condition")
endif()
```

&nbsp;  
&nbsp;  
&nbsp;  
# References
- [How to CMake good](https://youtu.be/_yFPO1ofyF0?list=PLK6MXr8gasrGmIiSuVQXpfFuE1uPT615s)
- [Effective Modern CMake](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1)
- [CMake and CUDA](https://developer.nvidia.com/gtc/2019/video/S9444/video)