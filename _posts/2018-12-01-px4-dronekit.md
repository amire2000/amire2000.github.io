---
layout: post
title: PX4 sitl and dronekit
categories: px4
tags: [sitl, gazebo, px4, dronekit]
---

## Dronekit
DroneKit-Python allows developers to create apps that run on an onboard companion computer.

The API communicates with vehicles over MAVLink

> Dronekit is most compatible with ardupilot stack, from version 2.2.0 they have basic support in px4 stack


## Connect and monitor vehicle

- Terminal1 (run simulation without gui)
```
HEADLESS=1 make px4_sitl gazebo_iris
```

- Terminal 2 (run the above code/ commands)
```bash
#use ipython or run script
```
~~~python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

connection_string = '127.0.0.1:14540'
vehicle = connect(connection_string, wait_ready=True)

print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
~~~

## Resource
- [px4 dev guide](https://dev.px4.io/en/robotics/dronekit.html)
- [Dronekit](http://python.dronekit.io/)