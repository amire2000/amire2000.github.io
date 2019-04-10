---
layout: post
title: ros2
categories: ros2
tags: [ros2, vscode]
---

# Install ROS2
- ROS2 crystal
- colcon
- ros bridge
- Install teleop package for example

### ROS2
- install crystal `https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/`
  
### Colcon
- Using Colcon to build packages
  ```
  sudo apt install python3-colcon-common-extensions
  ```
### Install bridge
```
sudo apt install ros-crystal-ros1-bridge
```
### Additional packages
  sudo apt install ros-crystal-teleop-twist-keyboard

  ros2 run teleop_twist_keyboard teleop_twist_keyboard


# Workspace and packages
## create ws
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2
colcon build
source install/local_setup.bash && source install/setup.bash
```

## create package
- Run command under workspace `src` folder
```
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp
```

### Package folder and files structure
- launch folder: Contains launch files
- src folder: Source files (cpp, python)
- CMakeLists.txt: List of cmake rules for compilation
- package.xml: Package information and dependencies

## compile
```bash
#from ws root
colcon build --symlink-install
#colcon build --symlink-install --packages-select <package_name>
#
source install/setup.bash 
# list packages
ros2 pkg list | grep my
my_package

```


Part 1- Publisher
```
source  /opt/ros/crystal/local_setup.bash
cd ros2_ws/src
ros2 pkg create topic_publisher_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
```