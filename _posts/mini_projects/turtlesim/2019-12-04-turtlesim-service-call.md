---
layout: post
title: Turtlesim service call
categories: [mini, turtlesim]
tags: [ros]
image: turtle_twist.jpg
public: True
description: Learn ROS basic with turtlesim, Service call
---
A ROS service is a client/server system

- It is synchronous. The client sends a requests, and blocks until it receives a response.
- A service is defined by a name, and a pair of messages. Request/Response msg

## Turtlesim Services

```bash
rosservice list
#
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
```

### get info
For example `/spawn` service
- Message Type: `turtlesim/Spawn`
- Message Args: `x y theta name`
  
```bash
rosservice info /spawn
#
Node: /sim
URI: rosrpc://dev:60673
Type: turtlesim/Spawn
Args: x y theta name
```

### call service
From command line
> use Tab Tab to auto complete message struct after service name
`rosservice call /spawn <Tab> <Tab>`

```bash
rosservice call /spawn "x: 2.0
y: 2.0
theta: 1.57
name: 'new'"
# Response
name: "new"
```
![](/images/2019-12-04-10-54-03.png)