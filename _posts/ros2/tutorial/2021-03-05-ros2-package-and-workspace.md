---
layout: post
title: ROS2 packages and workspace
categories: ros2
tags: [tutorial]
image: ros2.png
description: 
public: true
---

- ROS2 package?
- Contents of a package
- ROS2 Workspace
  - installation workspace
  - local workspace
- Build
- Create a package
- 

```
colcon build
source install/setup.bash
ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_python <package_name>
ros2 run <package name> <executable name>
```

# Package
package contain
- Nodes
- Services
- Actions
- Messages
- Build Information
- Package Description

## CMake Packages
- package.xml
- CMakelists.txt
  
## Python Package
- package.xml
- setup.cpg
- setup.py

# Workspace
workspace contain multiple packages

- ROS2 installation workspace

```bash
# install into installationworkspace
sudo apt install ros-foxy-
source /opt/ros/foxy/setup.bash
```
## local workspace / overlay
- workspace for own package or others  downloading for internet


## create local workspace
```bash
mkdir ros2_ws
mkdir ros2_ws/src
cd ros2_ws/src
# Create a package
ros2 pkg create --build-type ament_python my_package
ros2 pkg create --build-type ament_python --node-name my_node my_package
```
