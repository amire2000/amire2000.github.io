---
layout: post
title: Launch gazebo sdf model from ROS
categories: ros
tags: [101]
public: true
description: Using ROS launch to spawn sdf model and gazebo world
image: launch.png
---

## Lab steps
- Create pkg
  - Create folders
    - world
    - launch
- Create basic launch file
- Create basic world file
- `source` the package
- Run `ros launch`
- [Create sdf launch](#launch-file)

# Create pkg and file
```bash
# create pkg
catkin_create_pkg gazebo_101
```

```
gazebo_101/
├── CMakeLists.txt
├── launch
│   └── my_world.launch
│   └── sdf_launch.launch
├── package.xml
└── worlds
│   └── my_world.world
└── models
    └── my_model
        └── model.config
        └── model.sdf

```
&nbsp;  
&nbsp;  
# launch file
```xml
<?xml version="1.0"?>
<launch>
    <arg name="verbose" default="true"/>
    <arg name="paused" default="false"/>
    <arg name="world" default="$(find gazebo_101)/worlds/my_world.world"/>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="verbose" default="$(arg verbose)"/>
        <arg name="world_name" default="$(arg world)"/>    
        <arg name="paused" default="$(arg paused)"/>
        <arg name="use_sim_time" default="true"/>
    </include>
</launch>
```
&nbsp;  
&nbsp;  
# world file
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
	<world name="my_world">
		<!-- A global light source -->
		<include>
			<uri>model://sun</uri>
		</include>
		<!-- A ground plane-->
		<include>
			<uri>model://ground_plane</uri>
		</include>
	</world>
</sdf>
```

# run (empty world)
```bash
# Don't forget to source
roslaunch gazebo_101 my_world.launch
```
&nbsp;  
&nbsp;  
# launch model
"ROS Service Call" Robot Spawn Method

```
rosrun gazebo_ros spawn_model -h
```

- Add node to launch file
  - `spawn_model`
    - mandatory args:
      - -model
      - -file
      - -sdf
- Create another launch and use it from main launch or manually

```xml
<?xml version="1.0"?>
<launch>
    <arg name="file" default="$(find gazebo_101)/models/my_model/model.sdf"/>
    <arg name="name" default="test"/>
    <node pkg="gazebo_ros" type="spawn_model" name="spawn"
        output="screen"
        args="-model $(arg name) -file $(arg file) -sdf"
    />
</launch>
```

## run
-  Terminal 1 (main launch)

```
roslaunch gazebo_101 my_world.launch
```

-  Terminal 2 (sdf launch)

```
roslaunch gazebo_101 sdf_launch.launch
```
![](/images/2019-10-01-23-13-52.png)
&nbsp;  
&nbsp;  
&nbsp;  
&nbsp;  
# prerequisite
- install `gazebo_ros` package

```
sudo apt install ros-melodic-gazebo-ros
```

# Reference
- [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [Roslaunch tips for large projects](http://wiki.ros.org/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects)