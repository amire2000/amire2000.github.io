---
layout: post
title: Gazebo model plugin 101
categories: gazebo
tags: [plugin]
image: plug.png
description: Show gazebo model plugin basic and how to used it
public: true
---

#  Model plugin
Plugins allow complete access to the physical properties of models and their underlying elements (links, joints, collision objects)

# Content
- [Project files and folders](#project-structure)
- [Model](#model-source)
  - header
  - source
  - cmake / meson build
- [Sdf](#modelsdf)
  - Model sdf
  - World

&nbsp;  
&nbsp;  
&nbsp;  
# Project structure
```
├── bin
├── build
├── worlds
│   └── test.world
├── models
│   └── model_plugin
│       ├── model.config
│       └── model.sdf
├── plugins
│   ├── meson.build
│   └── model_plugin
│       ├── model_plugin.hpp
│       └── model_plugin.cpp
└── meson.build
```
&nbsp;  
&nbsp;  
&nbsp;  
# Model Source
### model_plugin.hpp
```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    class DemoPlugin : public ModelPlugin
    {
    public:
        DemoPlugin();
        ~DemoPlugin();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
        void OnUpdate();
    private:
        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;
    };
} // namespace gazebo

```

### model_plugin.cpp
```cpp
#include "model_plugin.hpp"

namespace gazebo
{
DemoPlugin::DemoPlugin() {}
DemoPlugin::~DemoPlugin() {}
void DemoPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
    this->_model = _parent;
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DemoPlugin::OnUpdate, this));
}

void DemoPlugin::OnUpdate()
{
    // gzmsg << "on update";
}

GZ_REGISTER_MODEL_PLUGIN(DemoPlugin)
} // namespace gazebo
```

### cmake
```bash
# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(model_plugin SHARED model_plugin.cpp)
target_link_libraries(model_plugin ${GAZEBO_libraries})
install(TARGETS model_plugin DESTINATION ${PROJECT_SOURCE_DIR}/bin)
```

### Build with meson
```python
project('gazebo_tutorial', 'cpp',
 version : '0.1',
 default_options : ['warning_level=3', 'cpp_std=c++11'])

gz = dependency('gazebo')

install_dir = meson.source_root() + '/bin'

mylib = library('my_plugin', sources : ['model_plugin.cpp'],
                dependencies : [gz],
                install: true,
                install_dir: [install_dir])
```

&nbsp;  
&nbsp;  
&nbsp;  
# Sdf
### model.sdf
```xml
<?xml version='1.0'?>
<sdf version='1.4'>
	<model name="model_plugin">
    <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>

      <plugin name="model_plugin" filename="libmodel_plugin.so"/>
    </model>
</sdf>
```

### test.world
```xml
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>
   <include>
      <uri>model://model_plugin</uri>
    </include>
  </world>
</sdf>
```

&nbsp;  
&nbsp;  
&nbsp;  
# Reference
-  [gazebo model plugin](http://gazebosim.org/tutorials?tut=plugins_model)