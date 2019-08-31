---
layout: post
title: Gazebo stand alone application
categories: gazebo
tags: [uav, maps worlds]
image: ggazebo.png
description: Implement Gazebo standalone application with cpp and python wrapper 
public: true
---

# Subscribe gazebo event
## world diagnostic
```bash
gz topic -l
/gazebo/default/atmosphere
/gazebo/default/diagnostics
...
#info
gz topic -i /gazebo/default/diagnostics
Type: gazebo.msgs.Diagnostics

Publishers:
	192.168.2.253:33571

# view
# gui viewer
gz topic -v /gazebo/default/diagnostics

# console viewer
gz topic -e /gazebo/default/diagnostics
gz topic -e /~/diagnostics
```

## Stand alone App 
- Implement the same functionality like echo (-e)

```cpp
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <iostream>

// Type: gazebo.msgs.Diagnostics

void cb(ConstDiagnosticsPtr &_msg)
{
  // Dump the message contents to stdout.
  std::cout << "message" << std::endl;
  std::cout << _msg->DebugString();
}
 

int main(int argc, char **argv)
{
    gazebo::client::setup(argc, argv);
    //creat node
    gazebo::transport::NodePtr node(
        new gazebo::transport::Node());
    node->Init();

    //Listener
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/diagnostics", cb);
    while (true){
        gazebo::common::Time::MSleep(10);
    }
    gazebo::client::shutdown();
    return 0;
}
```

### CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gz_tools)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}"

add_executable(dia gz_dia.cpp)
target_link_libraries(dia ${GAZEBO_LIBRARIES})
```

## Python bind using ctypes
- Convert the simple / sample application to class
- Wrap `C++` class with `C` functions
- Compile as shared obj
- Write python app 

### Step 1 - convert executable to lib
- Class file ()
 
```cpp
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <iostream>

// Type: gazebo.msgs.Diagnostics
class GDia
{
    private:
        gazebo::transport::NodePtr _node;
        gazebo::transport::SubscriberPtr _sub;

    public:
        GDia()
        {
            gazebo::client::setup();
            //creat node
            this->_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
            _node->Init();
        }

        void Run()
        {
             this->_sub = this->_node->Subscribe("/gazebo/default/diagnostics", this->_cb);
            while (true){
                gazebo::common::Time::MSleep(10);
            }
        }

        ~GDia()
        {
            gazebo::client::shutdown();
        }

        static void _cb(ConstDiagnosticsPtr &_msg)
        {
        // Dump the message contents to stdout.
        std::cout << "message" << std::endl;
        std::cout << _msg->DebugString();
        }
};

```

- main

```cpp
#include <iostream>
#include <gz_dia_w.cpp>

int main(int argc, char const *argv[])
{
    GDia dia;
    dia.Run();
    return 0;
}
```

- CMakeLists.txt

```bash
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gz_tools)

find_package(gazebo REQUIRED)
#gazebi include and current directory include
include_directories(${GAZEBO_INCLUDE_DIRS} .)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# shared lib
add_library(dia SHARED gz_dia_w.cpp)
target_link_libraries(dia ${GAZEBO_LIBRARIES})
#main
add_executable(gzmain main.cpp)
target_link_libraries(gzmain ${GAZEBO_LIBRARIES} dia)

```

### Step 2 - C Wrapper
> declare function pointer before cpp section or in header file

```c
extern "C"
{
    void (*cb2)(const char*);
    void (*cb)(int);
    void (*cb1)();
}
```

```c
extern "C"
{

    //return cpp object 
    GDia *GA_new() { return new GDia(); }

    const char* GA_version(GDia* obj)
    {
        obj->GetVer().c_str();
    }
    
    void GA_run(GDia* obj){
        obj->Run();
    }

    void GA_reg(void (*func)(const char*))
    {
        cb2 = func;
    }
}
```

### Step 3 - python
```python
import ctypes
import time
import threading

# load so files
lib = ctypes.cdll.LoadLibrary('/home/user/projects/gazebo_tutorial/tools/build/libdia.so')

class Foo(object):
    def __init__(self):
        lib.GA_new.restype = ctypes.c_longlong
        lib.GA_version.restype = ctypes.c_char_p
        self.obj = ctypes.c_longlong(lib.GA_new())

    def ver(self):
        return lib.GA_version(self.obj)

    def run(self):
        return lib.GA_run(self.obj)


    def register(self):
        c_cb = ctypes.CFUNCTYPE(None, ctypes.c_char_p)(cb1)
        lib.GA_reg(c_cb)

    def cb1(data):
    print ("before cb")
    print (type(data))
    print ("after cb")

if __name__ == "__main__":
    f = Foo()
    print(f.ver())
    f.register()
    print("----------")
    f.run()
```

# Reference
- [Con### Step 2 - trolling a Simulation External to Gazebo – Publishing](http://www.robopgmr.com/?p=5683)
- [publisher](https://bitbucket.org/osrf/gazebo/src/2e153b94b73a5f3e6301cc65640920cdb311f08c/examples/stand_alone/publisher/?at=default)
- [osrf gazebo examples](https://bitbucket.org/osrf/gazebo/src/default/examples/)