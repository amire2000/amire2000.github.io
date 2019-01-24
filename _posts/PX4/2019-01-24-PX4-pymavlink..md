---
layout: post
title: PX4  pymavlink
categories: PX4
tags: [px4, pymavlink]
---

```python
from pymavlink import mavutil, mavwp

import time
from threading import Thread

connection_string = '127.0.0.1:14540'

mav = mavutil.mavlink_connection('udp:' + connection_string)
mav.wait_heartbeat()
print("HEARTBEAT OK\n")

print ("ARM")
mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                          1,
                          0, 0, 0, 0, 0, 0)

msg = mav.recv_match(type=['COMMAND_ACK'],blocking=True)
print (msg)

PX4_MAV_MODE = 1
MAV_ACTION_LOITER = 27
mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                          PX4_MAV_MODE,
                          MAV_ACTION_LOITER, 0, 0, 0, 0, 0)
msg = mav.recv_match(type=['COMMAND_ACK'],blocking=True)
print (msg)

print ("Take off")
mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                         0, 1, 0, 0, 0, 3)
msg = mav.recv_match(type=['COMMAND_ACK'],blocking=True)
print (msg)
```

# Reference
- [donghee px4 offboard control](https://gist.github.com/donghee/15d2e890e745646a5458d92a244aef05)
- [Offboard Mode of Pixhawk](https://akshayk07.weebly.com/offboard-control-of-pixhawk.html)
- [subak](https://learn.subak.io/px4-workbook/px4-mavlink.html)