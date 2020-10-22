---
layout: post
title: Gazebo ign hello
categories: gazebo
tags: [ign]
description: Follow ign tutorial , build first ign gazebo robot
image: ignition_logo_color.svg
public: true
---

## import note
My graphics device driver not support OpenGL 3.3 needed by `ogre2` engine

All sdf need `gui` tag that set the engine to `orge`

See minimal world for example

## Minimal world
with `orge` engine settings

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
    <world name="world_demo">
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
### Physics

### Control and Status Plugins

### GUI Plugins

### environment variables
- `SDF_PATH`
- `IGN_FILE_PATH`
  

### Control From GUI
![](/images/2020-10-17-07-42-21.png)
&nbsp;  
&nbsp;  
&nbsp;  
## Empty World
[SDF World](https://ignitionrobotics.org/docs/citadel/sdf_worlds)

{% gist ad66b512f94a938e184683a592ebd0b4 %}

![](/images/2020-10-17-07-54-29.png)
  
# ign
The 'ign' command provides a command line interface to the ignition tools

## topic and services

&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Understanding the GUI](https://ignitionrobotics.org/docs/dome/gui)
- [Build your own robot](https://ignitionrobotics.org/docs/dome/building_robot)
- [Ignitionrobotics gazebo examples](https://github.com/ignitionrobotics/ign-gazebo/tree/main/examples)
- [Build a World](https://ignitionrobotics.org/docs/dome/sdf_worlds)