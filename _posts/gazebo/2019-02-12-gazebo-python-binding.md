---
layout: post
title: Gazebo python cpp bindings
categories: gazebo
tags: [python, cpp, bindings]
---

```python
import sys
sys.path.insert(0, "path to so")
import gazebo_bindings as g

def myf(data): 
    print (data)
    
g.subscriber(myf)
g.init()  
```

```cpp
#include <pybind11/pybind11.h>
#include <gazebo-9/gazebo/transport/TransportIface.hh>
#include <gazebo-9/gazebo/msgs/msgs.hh>
#include <gazebo-9/gazebo/gazebo_client.hh>

#include <iostream>
#include <csignal>

namespace py = pybind11;
py::object cb_func_ptr;

const std::string TOPIC = "/gazebo/default/world_stats";

void my_shutdown(int signal){
    std::cout<<"shutdown gazebo client" << std::endl;
    gazebo::client::shutdown();
    exit(0);
}

void cb(ConstWorldStatisticsPtr &_msg){
    //std::cout << _msg->DebugString() << std::endl;
    cb_func_ptr(_msg->DebugString());
}

void subscriber(const py::object& f){
    std::cout << "subscriber" << std::endl;
    cb_func_ptr = f;
}

void init(){
    std::cout << "innit" << std::endl;
    signal (SIGINT, my_shutdown);
    gazebo::client::setup();

    //create node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    //subscribe to topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe(TOPIC, cb);
     while(true){
        gazebo::common::Time::MSleep(100);
    }
}

PYBIND11_MODULE(gazebo_bindings, m) {
    // py::module m("gazebo_bindings");
    m.def("init", &init);
    m.def("subscriber", &subscriber);
    // return m.ptr();
}
```

- cmake
```cmake
cmake_minimum_required(VERSION 3.10)
project(cmake_example)

add_compile_options(-std=c++11)
set(PYBIND11_CPP_STANDARD -std=c++11)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(pybind11 REQUIRED)


add_library(gazebo_bindings MODULE src/gamain.cpp)
target_link_libraries(gazebo_bindings PRIVATE pybind11::module ${GAZEBO_LIBRARIES} pthread)
set_target_properties(gazebo_bindings PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")


```