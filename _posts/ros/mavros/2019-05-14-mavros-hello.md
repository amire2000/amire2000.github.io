---
layout: post
title: MAVROS hello world
categories: ros
tags: [ros, mavros, px4, ardupilot]
image: mavros.jpg
description: Mavros and px4 first step, PX4 iris gazebo simulation with SITL communicate and ROS with mavros package
public: true
---

## Content
- Install mavros
- Basic usage PX4 and SITL
- Using ros services: Takeoff and Land
  
# Install mavros
```
sudo apt install ros-melodic-mavros
sudo apt install ros-melodic-mavros-extras
```

# Basic usage PX4 and SITL 
- Run px4 SITL with iris gazebo simulation
- Run mavros with default connection (Offboard mavlink communication)
  - `fcu_url" default="udp://:14540@localhost:14557"/`
- Used pre defined apps to ARM, takeoff and land


### Terminals
- Terminal 1 (SITL and Gazebo)
```bash
make px4_sitl_default gazebo
```

- Terminal 2 (mavros)
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

- Termianl 3 (run commands)
  - mavXXX commands installed `/opt/ros/melodic/lib/mavros`
  
```bash
cd /opt/ros/melodic
# Set cuurent as a home position
lib/mavros/mavcmd sethome -c 0 0 0

#arm
lib/mavros/mavsafety arm

# takeoff current position
#takeoffcur pitch yaw altitude
lib/mavros/mavcmd takeoffcur 0 0 5

# land current position
# landcur yaw altitude
lib/mavros/mavcmd landcur 0 0
```

![](/images/2019-05-14-16-47-15.png)


&nbsp;  
# Using ros services: Takeoff and Land
- using mavros services
  - `/mavros/cmd/arming`
  - `/mavros/cmd/takeoff`
  - `/mavros/cmd/land`
> We need to fill real data in takeoff latitude and longitude arguments for hovering,
> Read the gps data from `/mavros/global_position/global` topic


```bash
# read gps data
rostopic echo -n1 /mavros/global_position/global
..
latitude: 47.3977511
longitude: 8.5456073

# Arming
rosservice call /mavros/cmd/arming "value: true"

# latitude and longitude must be real value
rosservice call /mavros/cmd/takeoff \
"{min_pitch: 0.0, yaw: 0.0, \
latitude: 47.3977511, \
longitude:  8.5456073, \
altitude: 5.0}"

# land
rosservice call /mavros/cmd/land \
"{min_pitch: 0.0, yaw: 0.0, \
latitude: 0.0, \
longitude: 0.0, \
altitude: 0.0}"
```

&nbsp;  
&nbsp;  
&nbsp;  

# SITL Network ports 
![](/images/2019-05-14-07-57-27.png)

&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [PX4 using ROS](http://dev.px4.io/en/simulation/ros_interface.html)