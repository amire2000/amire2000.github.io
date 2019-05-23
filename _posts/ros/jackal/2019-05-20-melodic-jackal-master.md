---
layout: post
title: Jackal ROS Course
categories: ros
tags: [navigation, jackal]
image: jackal.png
description: The construct course, jackal mastering ros create real world applications with Clearpath jackal robot
---

# Content
- Install ROS melodic

# Install 
- Clone repo from ``
- Clone `pointgray_camera_description` from [git](https://github.com/ros-drivers/pointgrey_camera_driver)
  - I Copy only camera_description folder to `catkin_ws`


# Jackal indoor 
```
roslaunch jackal_course_indoor main.launch 
```

![race jackal](/images/2019-05-20-22-10-17.png)

![rviz  move_base](/images/#%20Create%20a%20map
.png)

# Create a Map
To create a Map you need to launch two elements: GMapping and MoveBase.

- GMapping: in the one in charge of generating and saving the map. More info here.
- MoveBase: The one in charge of sending the commands to `/cmd_vel` to move the robot around based on sensors and now generating map.

```
roslaunch my_jackal_tools start_mapping.launch
```
![gmapping rviz](/images/2019-05-21-11-48-52.png)

navigate the robot with rviz 2D Nav Goal

save the map

```
roscd my_jackal_tools
mkdir maps
cd maps
rosrun map_server map_saver -f mymap
```

# Navigate with your map
- Fixed point: /map
- Map: /map
- Map: /move_base/global_costmap/costmap
rviz

![rviz navigation](/images/2019-05-21-13-45-21.png)

- Use 2D Pose Estimate to init robot start position
- When the laser scan align with the map then the robot is localize


# Add WayPoints
## install follow_waypoints
- clone from git

![follow_waypoint](/images/2019-05-21-14-00-20.png)

- Check if dependencies installed
```bash
rospack list | grep  actionlib
actionlib /opt/ros/melodic/share/actionlib
actionlib_msgs /opt/ros/melodic/share/actionlib_msgs
actionlib_tutorials /opt/ros/melodic/share/actionlib_tutorials
turtle_actionlib /opt/ros/melodic/share/turtle_actionlib
```

```bash
cd ~/catkin/src
git clone https://github.com/danielsnider/follow_waypoints.git
cd ..
catkin build
set_ws
#check
rospack list | grep follow
follow_waypoints /home/user/catkin_ws/src/follow_waypoints
```

rospack list | grep follow
follow_waypoints /home/user/catkin_ws/src/follow_waypoints