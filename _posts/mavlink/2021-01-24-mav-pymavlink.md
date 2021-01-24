---
layout: post
title: PyMavlink hello
categories: APM
tags: [mavlink, pymavlink, arducopter]
public: true
image: mavlink_logo.png
description: Using pymavlink to read and send MAVLink messages to ArduCopter
---

# Lab 1
- Read message stream

## env
- ArduCopter SITL ver 4.03


## Demo Code

```python
from pymavlink import mavutil

def handle_heartbeat(msg):
    print(msg)
    print(type(msg))

def read_loop(m):
    while(True):
        m.select(0.05)
        try:
            msg = m.recv_msg()
            
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    print(msg.data)
            elif msg_type == "HEARTBEAT":
                handle_heartbeat(msg)
            else:
                print(msg_type)
        except Exception as e:
            print(e)

if __name__ == "__main__":
    rate = 4
    start_sending = 1
    cs = "tcp:localhost:5760"
    #cs = "udp:127.0.0.1:14550"
    master = mavutil.mavlink_connection(cs)
    master.wait_heartbeat()
    # request data to be sent at the given rate
    master.mav.request_data_stream_send(master.target_system, 
        master.target_component, 
        mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, start_sending)
    print(f"heatbeat from {master.target_system}")
    read_loop(master)

```