---
layout: post
title: ROS2 PX4 bridge
categories: px4
tags: [ros2, rtps, dds]
public: true
image: px4.jpg
description: ROS2 PX4 bridge
---
from px4 docs:  
The PX4-Fast DDS Bridge, adds a Real Time Publish Subscribe (RTPS) interface to PX4, enabling the exchange of uORB messages between PX4 components and (offboard) Fast DDS applications. This allows us to better integrate with applications running and linked in DDS domains (including ROS nodes), making it easy to share sensor data, commands, and other vehicle information.

[PX4-Fast RTPS(DDS) Bridge](https://docs.px4.io/master/en/middleware/micrortps.html)

![](/images/2021-03-12-16-24-04.png)

# Demo LAB
[ROS World 2020: Getting started with ROS 2 and PX4 (2020)](https://youtu.be/qhLATrkA_Gw)
  


- Build px4 with rtps support
- Create `ROS2` workspace
  - Clone from git `px4_ros_com`
  - Clone from git `px4_msgs`
- Run micrortps_client on px4 side
- Run micrortps_agent on computer side
- Subscribe and Publish messages


## Setup

```bash
mkdir -p ~/px4_ros_com_ros2/src
cd ~/px4_ros_com_ros2/src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
# Build
./src/px4_ros_com/scripts/build_ros2_workspace.bash
```

## Run
### Terminal 1
- Run px4 sitl with rtps and gazebo as simulator
- Don't forget: Run rtps client
  
```bash
make px4_sitl_rtps gazebo
#Run Client
micrortps_client start -t UDP
```

### Terminal 2
- Run Agent

```bash
cd ~/px4_ros_com_ros2/src
source install/setup.bash
micrortps_agent -t UDP
```

### Terminal 3
- echo message from px4

```bash
cd ~/px4_ros_com_ros2/src
source install/setup.bash
ros2 topic list
#
/DebugVect_PubSubTopic
...
/SatelliteInfo_PubSubTopic
/SensorCombined_PubSubTopic
...

# from terminal 
ros2 topic echo /SensorCombined_PubSubTopic
# Result
---
timestamp: 1173986455999
gyro_rad:
- 0.0010652641067281365
- 0.0021305305417627096
- 0.0015978955198079348
gyro_integral_dt: 4000
accelerometer_timestamp_relative: 0
accelerometer_m_s2:
- -0.004788404330611229
- 0.0418987050652504
- -9.668983459472656
accelerometer_integral_dt: 4000
accelerometer_clipping: 0

```
&nbsp;  
&nbsp;  
&nbsp;  
## Lab 2
- publish message from ROS2 node

### Terminal 2
- Run Agent

> check that `DebugVect` subscriber are started

### Termianl 3 (ros node)

```
ros2 run px4_ros_com debug_vect_advertiser
```

### Terminal 1 (px4)
- from px4 terminal run `listener` command

```
listener debug_vect
```

&nbsp;  
&nbsp;  
&nbsp;  
# micrortps agent
- Download from [github repo](https://github.com/PX4/micrortps_agent) (Don't compile from px4_firmware build source)

```bash
git clone https://github.com/PX4/micrortps_agent
cd micrortps_agent
mkdir build
cd buid
cmake ..
make
```

# Reference
- [ROS World 2020: Getting started with ROS 2 and PX4 (2020)](https://youtu.be/qhLATrkA_Gw)
- [ROS World 2020: The roadmap to micro-ROS, closing the bridge between ROS 2 and PX4](https://youtu.be/8XRkzHqQSf0?list=PLk1TkXMY8_feC2L7aG2rp-FO31IzGWX_F)