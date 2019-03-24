---
layout: post
title: USB/ Serial connection to pixhawk
categories: pixhawk
tags: [pixhawk, serial, usb]
---

## Connect from ubuntu
> Remove modemmanger
```
sudo apt purge modemmanger
```

## Connect with mavproxy 
- QGC default port: 14550
```
mavproxy.py --master="/dev/ttyACM0" --out="udp:127.0.0.1:14550"
```