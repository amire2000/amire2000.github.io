---
layout: post
title: Gazebo wind plugin
categories: gazebo
tags: [plugins, wind]
public: true
description: Create custom wind plugin base on exists gazebo wind plugin, This post show basic steps to create gazebo world plugins and introduction to gazebo physics and other API
image: pinwheel.png
---

# Project structure
```
├── bin
├── build
├── lib
├── worlds
│   └── wind.world
├── plugins
│   ├── CMakeLists.txt
│   └── MyWind.cpp
├── CMakeLists.txt
└── init_env.sh
```

# wind.world
## SDF model specification
- enabled wind
  
![](/images/2019-08-04-21-21-07.png)

```xml
<?xml version="1.0"?>
<sdf version="1.4">
	<world name="default">
		<!-- Ground Plane -->
		<include>
			<uri>model://ground_plane</uri>
		</include>
		<include>
			<uri>model://sun</uri>
		</include>
		<!-- box -->
		<model name="box">
			<pose>0 0 0.5 0 0 0</pose>
			<enable_wind>True</enable_wind>
			<link name="box_link">
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
		</model>
		<plugin name="hello_world" filename="libMyWind.so"/>
	</world>
</sdf>
```
# wind plugin

## messages and debug
```cpp
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldPluginTutorial : public WorldPlugin
  {
    public: WorldPluginTutorial() : WorldPlugin()
            {
              gzdbg << "debug message" << std::endl;
              gzmsg << "message" << std::endl;
              gzwarn << "error message" << std::endl;
              gzerr << "error message" << std::endl;
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
```

![](/images/2019-08-04-21-37-57.png)


## Basic wind plug
```cpp
#include <gazebo/gazebo.hh>
#include "gazebo/physics/physics.hh"

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
private:
  event::ConnectionPtr updateConnection;
  physics::WorldPtr world;

public:
  WorldPluginTutorial() : WorldPlugin()
  {
    gzdbg << "debug message" << std::endl;
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    this->world = _world;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldPluginTutorial::OnUpdate, this));
  }

  void OnUpdate()
  {

    physics::Model_V models = this->world->Models();

    // Process each model.
    for (auto const &model : models)
    {
      // Get all the links
      physics::Link_V links = model->GetLinks();

      // Process each link.
      for (auto const &link : links)
      {
        // Skip links for which the wind is disabled
        if (!link->WindMode())
          continue;

        // Add wind velocity as a force to the body
        link->AddRelativeForce(link->GetInertial()->Mass() *
                               (link->RelativeWindLinearVel() - link->RelativeLinearVel()));
      }
    }
  }
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
} // namespace gazebo

```

# CMAKE
- main CMakeLists.txt
  - Set gazebo include and library and compiler flags
  - Set outputs folders
- Plugs CMakeLists.txt
  - Declare each plug

### main cmake
```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

add_subdirectory(plugins)
```

### plugs cmake
```cmake
add_library(MyWind SHARED hello_wind.cpp)
target_link_libraries(MyWind ${GAZEBO_LIBRARIES})
```

# Environment
## Gazebo
```
GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:<path to plugins>
```

# Usage
```
gazebo --verbose worlds/wind.world
```

![](/images/2019-08-04-21-43-36.png)

## Run
![](/images/wind_demo.gif)
&nbsp;  
&nbsp;  
&nbsp;  
# Misl.
## vscode tasks
- Create cmake task
- Create make task
  - depend on cmake tasks

{% gist 4db86922f7a5529cd626e55a59aa82fa %}