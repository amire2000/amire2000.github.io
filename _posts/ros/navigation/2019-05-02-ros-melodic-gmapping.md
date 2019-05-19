---
layout: post
title: ROS melodic install and use gmapping
categories: ros
tags: [navigation, slam, gmapping]
image: robot_maze.jpg
description: ROS SLAM Navigation 101 using gmapping
public: true
---

# gmapping
Implement SLAM navigation using gmapping algorithm

- Build a map
- Localization using amcl
- Path planning

> melodic distro don't support gmapping package
We need to install package from source  

![gmapping ros page](/images/2019-05-02-09-48-25.png)

## install from source
- check that all dependencies installed
  - for example
    ```bash
    rospack list | grep nav_msgs
    nav_msgs /opt/ros/melodic/share/nav_msgs
    ```
- Install missing or clone from source
> Check for packages at `repositories.ros.org/status_page/ros_melodic_default.html`

```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/openslam_gmapping
git clone https://github.com/ros-perception/slam_gmapping.git
cd ~/catkin_ws
catkin build
```

# Project
- kbot_description
- kbot_gazebo
- kbot_navigation

### kbot_description
Using `diff_wheeled_robot` robot from `mastering ROS for robotics` book, contain rviz launch file  

### kbot_gazebo
Using turtlebot playgroud world

### kbot_navigation
Has gmapping and navigation  launch files

# Create a map
- Terminal 1 (gazebo)
```bash
launch kbot_gazebo diff.launch
```

- Check topics and config gmapping params

```
/clock
/cmd_vel
/diff/scan
...
/initialpose
/joint_states
/map
/map_metadata
/map_updates
/move_base_simple/goal
/odom
..
/slam_gmapping/entropy
/tf
/tf_static

```
## gmapping important parameters
- scan: laser scan topic need to remap if topic name are not `scan`
- base_frame: The frame attached to the mobile base. same as `robotBaseFrame` declare at `libgazebo_ros_diff_drive.so` plugin
- odom_frame: The frame attached to the odometry system. same as `odometryTopic` declare at `libgazebo_ros_diff_drive.so` plugin
- map_frame
- maxRange: sensor max range
- maxUrange: usage range < maxRange

### gmapping launch file
```xml
<launch>

  <!-- used to map scan topic name -->
  <arg name="scan_topic" default="/diff/scan"/>

  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <remap from="scan" to="$(arg scan_topic)"/>
    
    <param name="odom_frame" value="odom"/>
    <param name="base_frame" value="base_footprint"/>
    <param name="map_frame" value="map"/>

    <param name="map_update_interval" value="1.0"/> <!-- default: 5.0 -->
    <param name="maxUrange" value="9.0"/>
    <param name="maxRange" value="10.0"/>
    <!-- more parameters -->
</launch>
```
- Terminal 2 (rviz)
```bash
launch kbot_description rviz.launch
```

- Terminal 3 (teleop)
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

- Terminal 4 (gmapping)
```bash
roslaunch kbot_navigation gmapping.launch 
```

- Map created after traveling the area with the robot
- Add map topic and laser scan to rviz
> Tip: change the :Laser scan `size` to view the scan more clearly

![gmapping map](/images/2019-05-03-01-35-04.png)


## Save the map
```
rosrun map_server map_saver -f $(rospack find kbot_navigation)/maps/playground_map
```
