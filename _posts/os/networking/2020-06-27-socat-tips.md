---
layout: post
title: SOCAT 
categories: os
tags: [network, tools]
image: 
description: 
public: true
---


# virtual serial port connection over TCP

Server - linux
Client - windows

- server: ser2net
- client: Virtual Serial Port (https://www.hw-group.com/software/hw-vsp3-virtual-serial-port)


```
socat TCP-LISTEN:4161,fork,reuseaddr FILE:/dev/ttyUSB0,b57600,raw

```


# Reference
- [Connecting to a Remote Serial Port over TCP/IP](https://www.acmesystems.it/socat)