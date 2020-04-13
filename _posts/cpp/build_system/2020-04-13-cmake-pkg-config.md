---
layout: post
title: Using pkg-config with cmake
categories: cpp
tags: [build, cmake, pkg-config]
public: true
description: Using CMake with pkg-config to build zmq
image: cmake.png
---

# LAB
Build zmq cpp application with cmake
- [using pkg-config cmake macros](#using-pkg-config)
- [install cppzmq](#install-zmq)
- [build using cmake find_package](#build-with-cppzmq)

&nbsp;  
&nbsp;  
&nbsp;  
## pkg-config
The pkg-config program is used to retrieve information about installed libraries in the system
It is typically  used  to compile and link against one or more libraries

```
g++ client.cpp $(pkg-config --cflags --libs libzmq)
```
&nbsp;  
&nbsp;  
## `pc` files
pkg-config look for `.pc` files to get metadata about the library  
for example: `/usr/lib/x86_64-linux-gnu/pkgconfig/libzmq.pc`
```
prefix=/usr
exec_prefix=${prefix}
libdir=${prefix}/lib/x86_64-linux-gnu
includedir=${prefix}/include

Name: libzmq
Description: 0MQ c++ library
Version: 4.2.5
Libs: -L${libdir} -lzmq
Libs.private: -lstdc++  -lsodium -lpgm -lpthread -lm -lnorm
Cflags: -I${includedir} 
```

The `pc` file installed by the package library


```bash
apt-file list libzmq3-dev
#
libzmq3-dev: /usr/include/zmq.h
libzmq3-dev: /usr/include/zmq.hpp
libzmq3-dev: /usr/include/zmq_utils.h
libzmq3-dev: /usr/lib/x86_64-linux-gnu/libzmq.a
libzmq3-dev: /usr/lib/x86_64-linux-gnu/libzmq.so
libzmq3-dev: /usr/lib/x86_64-linux-gnu/pkgconfig/libzmq.pc
...
```
&nbsp;  
&nbsp;  
&nbsp;  
## cmake
- find_package
- using pkg-config
- find_library

### find_package
libzmq came without `cmake file` 
> Install `cppzmq` add `cppzmqConfig.cmake` file  
> Check [build-with-cppzmq](#build-with-cppzmq)

&nbsp;  
&nbsp;  
### using pkg-config
- `pkg_search_module` macro
- Check [FindPkgConfig](https://cmake.org/cmake/help/v3.12/module/FindPkgConfig.html) for full arguments and settings

### pkg-config cmake variables
```
<XXX>_FOUND          ... set to 1 if module(s) exist
<XXX>_LIBRARIES      ... only the libraries (without the '-l')
<XXX>_LINK_LIBRARIES ... the libraries and their absolute paths
<XXX>_LIBRARY_DIRS   ... the paths of the libraries (without the '-L')
<XXX>_LDFLAGS        ... all required linker flags
<XXX>_LDFLAGS_OTHER  ... all other linker flags
<XXX>_INCLUDE_DIRS   ... the '-I' preprocessor flags (without the '-I')
<XXX>_CFLAGS         ... all required cflags
<XXX>_CFLAGS_OTHER   ... the other compiler flags
```

```cmake
find_package(PkgConfig REQUIRED)
pkg_search_module(ZeroMQ 
    REQUIRED
    libzmq
    IMPORTED_TARGET)

add_executable(server server.cpp)
# with IMPORTED_TARGET 
target_link_libraries(server PkgConfig::ZeroMQ)
# without IMPORTED_TARGET 
target_link_libraries(server ${ZeroMQ_LDFLAGS})
```

&nbsp;  
&nbsp;  
&nbsp;  
### find_library

```cmake
find_library(
    ZMQ_LIBRARY
    libzmq.so
    )
add_executable(server server.cpp)
target_link_libraries(server ${ZMQ_LIBRARY})
```

## build with cppzmq
- using find_package installed by cppzmq


```cmake
find_package(cppzmq)
add_executable(client client.cpp)
target_link_libraries(client cppzmq)
```
&nbsp;  
&nbsp;  
&nbsp;  
# Notes
## install zmq
- libzmq3-dev
- cppzmq (Install from [github](https://github.com/zeromq/cppzmq))


```
apt-get install libzmq3-dev
```

&nbsp;  
&nbsp;  
## pkg-tips
### Debug / Verbose
using  `PKG_CONFIG_DEBUG_SPEW` to get pkg-config verbose info

```bash
# Verbose
export PKG_CONFIG_DEBUG_SPEW=1; pkg-config --libs libzmq

# unset
unset PKG_CONFIG_DEBUG_SPEW
```

### zmq client req/rep pattern
- libzmq version (cppzmq has additional api)
  
```cpp
#include <string>
#include <iostream>
#include <zmq.hpp>

int main ()
{
    //  Prepare our context and socket
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REQ);

    std::cout << "Connecting to hello world server…" << std::endl;
    socket.connect ("tcp://localhost:5555");

    //  Do 10 requests, waiting each time for a response
    for (int request_nbr = 0; request_nbr != 10; request_nbr++) {
        zmq::message_t request (5);
        memcpy (request.data (), "Hello", 5);
        std::cout << "Sending Hello " << request_nbr << "…" << std::endl;
        socket.send (request);

        //  Get the reply.
        zmq::message_t reply;
        socket.recv (&reply);
        std::cout << "Received World " << request_nbr << std::endl;
    }
    return 0;
}
```
&nbsp;  
&nbsp;  
&nbsp;  
# References
- [Install ZMQ](https://zeromq.org/download/)
- [ZMQ C++](https://zeromq.org/languages/cplusplus/#cppzmq)
- [Guide to pkg-config](https://people.freedesktop.org/~dbn/pkg-config-guide.html)