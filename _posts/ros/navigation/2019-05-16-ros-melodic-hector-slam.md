---
layout: post
title: Hector SLAM
categories: ros
tags: [hector, slam, navigation]
image: logo_team_hector_black_h200px.png
public: false
description: Hector SLAM is an algorithm which can localize and map with only a lidar
---
Hector_slam contains ROS packages related to performing SLAM base on LIDAR output

# Content
- Compile hector slam for melodic
- Mapping with hector slam

# Compile hector slam for melodic
- Clone from git
- catkin build ( I got an error)
    - Got compile error hector_geotiff(QT4 missing)
![](/images/2019-05-16-20-46-46.png)

- Check [ROS Index](https://index.ros.org) for package status and dependencies

![](/images/2019-05-16-20-51-58.png)
  - build type: catkin
  - VCS Version: melodic-devel
  - Release: unreleased
- Check for dependencies
  - Packages dependencies
  - System dependencies
    - libqt4-dev

![](/images/2019-05-16-20-56-41.png)

- install `libqt4-dev` apt command `sudo apt install libqt4-dev`
- Compile again


# prepared robot
`rosrun rqt_tf_tree rqt_tf_tree`

![](/images/2019-05-17-06-50-03.png)

# Mapping with hector slam
hector_mapping is a node for LIDAR (sensor_msgs/LaserScan) based SLAM with no odometry


## Lets mapping
- Launch robot that publish `sensor_msgs/LaserScan` message.
- Launch hector_slam launch helper
  - Set the map file location
  - Run rviz
  - Run hector_mapping with `scan_topic` mapping
  - Run hector_geotiff mapper

### hector_mapping
[hector_mapping](http://wiki.ros.org/hector_mapping) is a SLAM approach that can be used without odometry as well as on platforms that exhibit roll/pitch motion

### hector_geotiff
[hector_geotiff](http://wiki.ros.org/hector_geotiff) provides a node that can be used to save occupancy grid map, robot trajectory and object of interest data to RoboCup Rescue compliant GeoTiff images

### hector launch file
```xml
<?xml version="1.0"?>

<launch>

  <arg name="geotiff_map_file_path" default="$(find kbot_navigation)/maps"/>

  <param name="/use_sim_time" value="true"/>

  <node pkg="rviz" type="rviz" name="rviz"
    args="-d $(find hector_slam_launch)/rviz_cfg/mapping_demo.rviz"/>

  <include file="$(find hector_mapping)/launch/mapping_default.launch">
    <arg name="odom_frame" default="base_footprint"/>
    <arg name="scan_topic" default="/diff/scan"/>
  </include>

  <include file="$(find hector_geotiff)/launch/geotiff_mapper.launch">
    <arg name="trajectory_source_frame_name" value="scanmatcher_frame"/>
    <arg name="map_file_path" value="$(arg geotiff_map_file_path)"/>
  </include>

</launch>

```

### save
```
rostopic pub syscommand std_msgs/String "savegeotiff"
```

## Terminals
- Termianl1 (run robot)
```bash

```

- Termianl2 (run hector (hector launch file) )
```bash

```

- Termianl1 (run teleop)
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

- Termianl4 (save)
```bash
rostopic pub syscommand std_msgs/String "savegeotiff"
```


> Tip: view image files from command line with `eog` Eye of Gnome for example
> `eog image.tif`



# Resources
1. [hector slam git](https://github.com/tu-darmstadt-ros-pkg/hector_slam)
2. [ROS wiki](http://wiki.ros.org/hector_slam)