---
layout: post
title: ROS Services 101
categories: ros
tags: [rospy, tutorial, services]
image: service.png
description: ROS Service allow for "request/ response" communication between nodes 
public: 1
---
# Content
- Service
- [Service message](#service-message)
- [Service Server](#server)
- [Service Client](#client)

# Service
ROS Service is like RPC , Request/ Response response communication between node client to node server, which defined by pair of messages
`request` and `response`
- **service server**: ROS Node "advertise" a service
- **Service client**: ROS Node "call" a service

> ROS Service block the program flow

# Service utils
## rosservice
- list
- call

## rossrv
provide information related to ROS Service definition
- list
- show
# Service message
- Create `AddTowInts.srv` file under `srv` folder
- Modify `package.xml`
- Modify `CMakeLists.txt`
- build and check

```bash
#request
int32 arg1
int32 arg2
---
#response
int32 result
```

## package.xml
- Add two lines 
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

## CMakeLists.txt
- Add `add_service_files` section
- Add `message_generation` to `find_package` section
```
cmake_minimum_required(VERSION 2.8.3)
project(py_tutorial)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_service_files(
  FILES
  AddTwoInts.srv
)

catkin_package(
  DEPENDS message_runtime
)
```

## build and check
```
catkin build py_tutorial
```
- rossrv list
```bash
rossrv list | grep Add
#
py_tutorials/AddTwoInts
```

- rossrv show
```
rossrv show py_tutorials/AddTwoInts
int64 a
int64 b
---
int64 sum
```

# Server
- code
- usage / run
## code

{% gist 97103e9decf5d40990b47274b88d4dfa %}

## run
- Terminal 1
```
roscore
```

- Terminal 2 (run service server)
```
rosrun py_tutorial service_server.py 
[INFO] [1558704151.531481]: server service is now available.
```

```
rosservice list | grep add

/addInts_server/get_loggers
/addInts_server/set_logger_level
/add_ints
```

- call
```bash
# rosservice call /add_ints TAB TAB"
 rosservice call /add_ints "arg1: 1
arg2: 2" 
result: 3
```


# Client
## code
{% gist 99545a7dd0d24e0f891bee0ad4b3c21e %}


## run / usage
- terminal 1 (roscore)
```bash
roscore
```

- terminal 2 (ros service)
```bash
rosrun py_tutorial service_server.py 
[INFO] [1558704151.531481]: server service is now available.
```

- terminal 3 (ros client)
```bash
rosrun py_tutorial service_client.py 
[INFO] [1558704868.898275]: Waiting for service...
[INFO] [1558704868.911881]: calc result: 3!
```

# Reference
- [ROS Services code illustration](https://ocw.tudelft.nl/course-lectures/1-3-2-ros-services-code-illustration-service-server-part-1/)
- [ROS catkin CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)
