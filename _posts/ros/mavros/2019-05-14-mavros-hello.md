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

# Code sample
Python node to get UAV state and arming
- Subscribe to `/mavros/state` topic
- Using `/mavros/cmd/arming` to ARM.

```python
#!/usr/bin/env python
"""
mavros px4 demo
"""
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

current_state = None

class OffboardNode(object):
    def __init__(self):
        self._current_state  = State()
        self._rate = rospy.Rate(20)

        self._state_sub = rospy.Subscriber("/mavros/state",
            State,
            self.state_cb)

        self._arming_srv = rospy.ServiceProxy("/mavros/cmd/arming",
            CommandBool)

        # wait for connection
        while not rospy.is_shutdown() and not self._current_state.connected:
            self._rate.sleep()
            rospy.loginfo("Wait for connection")
        
        rospy.loginfo("Connected")


    def state_cb(self, msg):
        """
        connected: True
        armed: False
        guided: True
        mode: "AUTO.LOITER"
        system_status: 3
        """
        rospy.loginfo(msg)
        self._current_state = msg

    def arming(self):
        try:
            response = self._arming_srv(True)
            rospy.loginfo(response)
            if not response.success:
                raise Exception("Failed to ARMING")

        except rospy.ServiceException, e:
            rospy.logerr("Failed to ARMED")
            raise


    def run(self):
        self.arming()

        while not rospy.is_shutdown():
            self._rate.sleep()

def main():
    rospy.init_node("offboard")
    node = OffboardNode()
    node.run()


if __name__ == "__main__":
    main()
```
&nbsp;  
&nbsp;  
&nbsp;  

## State message 
```
[INFO] [1557908111.694454]: header: 
  seq: 3026
  stamp: 
    secs: 1557908111
    nsecs: 693815730
  frame_id: ''
connected: True
armed: True
guided: True
mode: "AUTO.LOITER"
system_status: 4
```

> Note: check way mode not changed to OFFBOARD

# SITL Network ports 
![](/images/2019-05-14-07-57-27.png)

&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [PX4 using ROS](http://dev.px4.io/en/simulation/ros_interface.html)