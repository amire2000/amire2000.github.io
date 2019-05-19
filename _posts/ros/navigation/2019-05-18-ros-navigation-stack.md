---
layout: post
title: ROS Navigation stack , Hello
categories: ros
tags: [navigation stack, gmapping]
---
# Content
- Setup environment
- Run gmapping
  - Save the map


## launch world and spawn robot

```xml
<launch>
  <arg name="paused" default="false"/>
  <arg name="world_name" default="$(find jackal_gazebo)/worlds/jackal_race.world"/>
  
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(arg world_name)" />
    <arg name="verbose" value="true"/>
    <arg name="paused" value="$(arg paused)"/>
  </include>

  <!-- urdf xml robot description loaded on the Parameter Server-->
  <param name="robot_description" 
  command="$(find xacro)/xacro '$(find kbot_description)/urdf/diff_wheeled_robot.xacro'" /> 

  	<param name="use_gui" value="false"/>
	<!-- Starting Joint state publisher node which will publish the joint values -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

	<!-- Starting robot state publish which will publish tf -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
  

  Run a python script to the send a service call to gazebo_ros to spawn a URDF robot
  <node name="urdf_spawner" 
  pkg="gazebo_ros" 
  type="spawn_model" 
  respawn="false" output="screen"
  args="-urdf -model diff_wheeled_robot -param robot_description"/> 
</launch>

```


## gmapping
- Set scan_topic (LaserScan topic)
- odom_frame
- base_frame
- map_frame (publish)
```xml
<launch>

  <!-- used to map scan topic name -->
  <arg name="scan_topic" default="/diff/scan"/>

  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <remap from="scan" to="$(arg scan_topic)"/>
    
    <param name="odom_frame" value="odom"/>
    <param name="base_frame" value="base_footprint"/>
    <param name="map_frame" value="map"/>

    <!-- Process 1 out of every this many scans (set it to a higher number to skip more scans)  -->
    <param name="throttle_scans" value="1"/>

    <param name="map_update_interval" value="1.0"/> <!-- default: 5.0 -->

    <!-- The maximum usable range of the laser. A beam is cropped to this value.  -->
    <param name="maxUrange" value="9.0"/>

    <!-- The maximum range of the sensor. If regions with no obstacles within the range of the sensor should appear as free space in the map, set maxUrange < maximum range of the real sensor <= maxRange -->
    <param name="maxRange" value="10.0"/>

    ....
```

## teleop
```xml
<?xml version="1.0"?>
<launch>
  <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop_twist_keyboard"  output="screen">
    <param name="scale_linear" value="1.0" type="double"/>
    <param name="scale_angular" value="1.5" type="double"/>
  </node>
</launch>
```


# Running
- Terminal 1
  - gazebo world and robot
- Terminal 2
  - teleop
- Termianl 3
  - gmapping
- Termianl 4
  - run rviz
- Terminal 5 (save map)
  - from map directory
  - `rosrun map_server map_saver -f my_map`

## Rviz
- Add `LaserScan`
  - topic: /diff/scan
  - size (laser marker): 0.1 (display laser more clearly)
- Add `Map`
  - topic: `/map` (published by `/slam_gmapping` node)

![](/images/2019-05-18-23-54-53.png)


# Reference
- [ROS Developers LIVE-Class #13: ROS Navigation Stack How To](https://www.youtube.com/watch?v=fTizQneURWo&t=3675s)