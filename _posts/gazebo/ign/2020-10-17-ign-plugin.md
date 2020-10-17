---
layout: post
title: Gazebo ign plugin hello
categories: gazebo
tags: [ign]
description: ignition plugin
image: ignition_logo_color.svg
public: true
---
In Ignition Gazebo, all systems are loaded as plugins at runtime

## Project 
```
└── worlds
    └── empty_plug.sdf
└───plugins
    ├── build
    ├── CMakeLists.txt
    ├── SampleSystem.cc
    └── SampleSystem.hh
```
## Header
- class inherit from  `ignition::gazebo::System`
- Inherit other interfaces (Check reference for more info)
  - ISystemConfigure
  - ISystemPreUpdate
  - ISystemUpdate
  - ISystemPostUpdate
- Register the plug with 

# Code
Ignition header location (install from `deb`) `/usr/include/ignition/`
- SampleSystem.hh
  
```cpp
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>

namespace sample_system
{
  class SampleSystem:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPostUpdate
  {
    public: SampleSystem();
    public: ~SampleSystem() override;
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
  };
}

IGNITION_ADD_PLUGIN(
    sample_system::SampleSystem,
    ignition::gazebo::System,
    sample_system::SampleSystem::ISystemPostUpdate)
```

## Code
- SampleSystem.cc

```cpp
#include "SampleSystem.hh"

using namespace sample_system;
SampleSystem::SampleSystem()
{
}
SampleSystem::~SampleSystem()
{
}
void SampleSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem::PostUpdate" << std::endl;
}
```


## Cmake
- CMakeLists.txt
  
```c
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(SampleSystem)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo4 REQUIRED)

add_library(SampleSystem SHARED SampleSystem.cc)
set_property(TARGET SampleSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(SampleSystem
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo4::ignition-gazebo4)
```

## World

- empty_plug.sdf

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
    <world name="world_demo">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics">
        </plugin>
        <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands">
        </plugin>
        <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>
        <plugin filename="libSampleSystem.so" name="sample_system::SampleSystem">
        </plugin>
        <gui fullscreen="0">
            <!-- 3D scene -->
            <plugin filename="GzScene3D" name="3D View">
                <ignition-gui>
                    <title>3D View</title>
                    <property type="bool" key="showTitleBar">false</property>
                    <property type="string" key="state">docked</property>
                </ignition-gui>

                <engine>ogre</engine>
                <scene>scene</scene>
                <ambient_light>0.4 0.4 0.4</ambient_light>
                <background_color>0.8 0.8 0.8</background_color>
            </plugin>
            
        </gui>
        </world>
</sdf>
```

## Usage
> Set `GN_GAZEBO_SYSTEM_PLUGIN_PATH` env variable

- Terminal
  
```bash
#export
export GN_GAZEBO_SYSTEM_PLUGIN_PATH <so location>
ign gazebo -v 4 empty_plug.sdf
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Create System Plugins](https://ignitionrobotics.org/api/gazebo/4.0/createsystemplugins.html)
