---
layout: post
title: Gazebo sensors  and plugins
categories: gazebo
tags: [gazebo, plugins, sensors]
---
A Plugin is a piece of code that use to change things in a simulation
Gazebo have different type of plugins
- Model plugin: controls a model
- World: controls the world
- System plugin:
- Sensor plugin:

A Sensor is piece of code that retrieve information from  the simulation environment (like camera,gps, imu), with sensor plugin we control the sensor behavior.
> Sensors source code: `gazebo/sensors`



## Reference
- [Creating a light sensor plugin](https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/wiki/Sec.-4:-Creating-a-light-sensor-plugin)
                                                                                                                                                                                                                                                                                            
# Sensor plugin
```cpp
#include <gazebo/gazebo.hh>

namespace gazebo
{
class GAZEBO_VISIBLE MySensorPlugin : public SensorPlugin
{
  public:
    MySensorPlugin();
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
};
GZ_REGISTER_SENSOR_PLUGIN(MySensorPlugin)
} // namespace gazebo
```

- CMake
```
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(hello_sensor SHARED hello_sensor.cc)
target_link_libraries(hello_sensor ${GAZEBO_LIBRARIES})
```