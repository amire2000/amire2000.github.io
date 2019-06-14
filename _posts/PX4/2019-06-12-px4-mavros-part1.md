---
layout: post
title: PX4 mavros
categories: px4
tags: [mavros]
public:  True
image: ros.png
description: Install and config PX4 to work with MAVROS
---
#Content
- Install GAAS Environment
- Setup Environment
- First run
- Offboard demo


# Install GAAS Environment
- Install ROS Melodic (apt)
- INSTALL PX4 Firmware (git)
- Install MAVROS (apt)
- Install other code dependencies


```
pip install pyquaternion
```

# Setup Environment
- Set GAAS and PX4 environment variables
  
### .bashrc
  
```
source /opt/ros/melodic/setup.bash
source ~/px4/Firmware/Tools/setup_gazebo.bash ~/px4/Firmware/ ~/px4/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/px4/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/px4/Firmware/Tools/sitl_gazebo
```

# First RUN
- Terminal 1 (gazebo)
```bash
roslaunch px4 posix_sitl.launch
```

- Terminal 2 (mavros)
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
# roslaunch px4 mavros_posix_sitl.launch
```
![](/images/2019-06-14-09-57-40.png)


- Teminale 3
  - check state
  - check that `connected` argument are `true`


```bash
rostopic echo /mavros/state
# output
---
header: 
  seq: 80
  stamp: 
    secs: 179
    nsecs: 532000000
  frame_id: ''
connected: True
armed: False
guided: True
mode: "AUTO.LOITER"
system_status: 3
---
```

# OFFBOARD Control
## Demo1
- Download code from git
- Run Gazebo and mavros


- Terminal 1 (gazebo)
```bash
roslaunch px4 posix_sitl.launch
```

- Terminal 2 (mavros)
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

- Terminal 3 (git code )
```bash
cd GAAS/demo/tutorial_1/1_px4_mavros_offboard_controller
python px4_mavros_run.py
```

- Terminal 4 (same folder as Terminal3)
```python
# code from https://gaas.gitbook.io/guide/
# import packages
from commander import Commander
import time

# create Commander instance
con = Commander()

# control the drone to move 1 meter to the right
con.move(1,0,0)
# wait for 2 seconds 
time.sleep(2)

# control the drone to move 1 meter to the front
con.move(0,1,0)

# land
con.land()
```

# Reference Frames
- ENU: 
    - x: East
    - y: North
    - z: Up

- NED:
    - x: North
    - y: East
    - z: Down

- BODY_OFFSET_ENU: 
- BODY_OFFSET_FLU: Forward, Left, Up, Move relative to its body frame

- LOCAL_ENU:  movement is relative to its takeoff position





# Reference
- [GAAS](https://gaas.gitbook.io/guide/)
- [GAAS Git](https://github.com/generalized-intelligence/GAAS)