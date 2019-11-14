---
layout: post
title: Follow the line - part 01
categories: [mini, line]
tags: [ros, opencv, gazebo, urdf]
public: true
description: Mini project implement mini bot to tracking line on ground, using opencv gazebo and ros
image: follow_line.jpeg
---

# LAB1 - Robot and Environment
- Create catkin w.s
- Create urdf package `cart_description`
  - urdf
    - using xacro
  - launch files
    - spawn rviz
- Create sim package `cart_sim`
  - launch files
    - spawn model
  
# Content

- [cart_description](#cartdescription)
- [cart_sim](#cartsim)

&nbsp;  
&nbsp;  
&nbsp;  
## Create workspace

```bash
mkdir -p sim_ws/src
cd sim_ws
catkin_make
```
&nbsp;  
&nbsp;  
&nbsp;  
# cart_description
## Create `cart_description` package

```bash
cd src
catkin_create_pkg cart_description urdf
```

- Folders
```
├── CMakeLists.txt
├── launch
│   ├── gazebo.launch
│   └── rviz.launch
├── package.xml
├── rviz
│   └── config.rviz
└── urdf
    ├── cart.gazebo
    ├── cart.xacro
    ├── macros.xacro
    └── materials.xacro
```
&nbsp;  
&nbsp;  
&nbsp;  
## Create urdf
- `cart.xacro`: main urdf file
- `materials.xacro`: urdf materials definitions
- `cart.gazebo`: gazebo materials color and plugins
- `macros.xacro`: xacro helper macros

### Tips
- urdf validation

```bash
#cmd
check_urdf <(xacro cart.xacro)

#result
robot name is: cart
---------- Successfully Parsed XML ---------------
root Link: link_chassis has 3 child(ren)
    child(1):  link_left_wheel
    child(2):  link_right_wheel
    child(3):  sensor_camera

```

- Save xacro as urdf
```bash
xacro cart.xacro > cart.urdf
```


&nbsp;  
## launch files

- `rviz.launch`: launch `rviz` with robot and camera view

### rviz.launch
```xml
<?xml version="1.0"?>
<launch>
	<param name="robot_description" command="$(find xacro)/xacro '$(find cart_description)/urdf/cart.xacro'"/>
	<!-- send fake joint values -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
		<param name="use_gui" value="false"/>
	</node>
	<!-- Combine joint values -->
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
	<!-- Show in Rviz   -->
	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find cart_description)/rviz/config.rviz" />
</launch>
```

&nbsp;  
&nbsp;  
![](/images/2019-11-08-17-34-02.png)
&nbsp;  
&nbsp;  
========================================
&nbsp;  
&nbsp;  
# cart_sim

- Folders

```
├── CMakeLists.txt
├── launch
│   └── gazebo.launch
├── media
│   └── materials
│       ├── scripts
│       │   └── line.material
│       └── textures
│           └── course.png
├── models
│   └── line_plane
│       ├── materials
│       │   ├── scripts
│       │   │   └── line.material
│       │   └── textures
│       │       └── line.png
│       ├── model.config
│       └── model.sdf
├── package.xml
├── src
└── worlds
    └── follow.world
```

- `gazebo.launch`: launch empty world with the robot model
  - spawn model
  - run teleop node


&nbsp;  
![](/images/2019-11-08-17-32-35.png)




### Run gazebo without ROS
- set `GAZEBO_MODEL_PATH` variable before run
  
```bash
# Run from root package folder
cd cart_sim
export GAZEBO_MODEL_PATH=$(pwd)/models:$GAZEBO_MODEL_PATH && gazebo --verbose worlds/follow.world
```

### gazebo.launch
- load `follow` line world

```xml
<launch>
      <env name="GAZEBO_MODEL_PATH" value="$(find cart_sim)/models/" />
    <param name="robot_description" 
    command="$(find xacro)/xacro '$(find cart_description)/urdf/cart.xacro'" />

    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0.5"/>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
      <arg name="world_name" value="$(find cart_sim)/worlds/follow.world"/>
      <arg name="verbose" value="true" />
    </include>

    <node name="mybot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
          args="-urdf -param robot_description -model cart -x $(arg x) -y $(arg y) -z $(arg z)" />

    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop">
  	</node>
</launch>
```

> Point to world file full path using `$(find cart_sim)/worlds/follow.world`

> Set environment variable with `env` tag
```bash
<env name="GAZEBO_MODEL_PATH" value="$(find cart_sim)/models/" />
 ```

### Load model texture
- Add material to `visual` node
- Using `model` prefix to load texture declare at model scope

```xml
<material>
    <script>
        <uri>model://line_plane/materials/scripts</uri>
        <uri>model://line_plane/materials/textures</uri>
        <name>course/line</name>
    </script>
</material>
```

#### script file
- `line.material` file

```
material course/line
{
    technique
    {
        pass
        {
            texture_unit
            {
                texture line.png
            }
        }
    }
}
```

![](/images/2019-11-14-23-27-47.png)

# Reference
- [Exploring ROS using a 2 Wheeled Robot #1: Basics of Robot Modeling using URDF](https://www.theconstructsim.com/exploring-ros-2-wheeled-robot-part-01/)
- [line_follower_turtlebot](https://github.com/sudrag/line_follower_turtlebot)