---
layout: post
title: PX4 first step uORB
categories: px4
tags: [px4, uorb, nsh]
---

# PX4
## PX4 Architecture
The flight control run Nuttx system.
> NuttX: Real time operation system

The system  initialization according to NSH script. The script start essential modules/ applications at boot.  
each module/application are dedicated for specific task.  
For multi-copter for  example
- mc_att_control: attitude control
- mc_pos_control: position control

> Attitude control is controlling the orientation of an object  with respect to another entity

The modules send messages between them
using publish subscriber pattern (uORB)

The uORB application it's
self load at the begging of the rcS file in `init.d` folder.

- rcS
```bash
#
# Start the ORB (first app to start)
# tone_alarm and tune_control
# is dependent.
#
uorb start

```

![](/images/2019-01-05-13-19-16.png)

# uORB
The uORB is an asynchronous publish() / subscribe() messaging API used for inter-thread/inter-process communication
> uORB: Micro Object Request Broker

## uORB terminology
-  node: process
-  topic: message bus
-  Advertising: 
-  publish: Send a message
-  subscribing: 

![](/images/2019-01-05-13-21-39.png)

The  communication message are data struct and send on topic (message  bus)  
Modeule/application can  subscribe to `topic` from uORB and `Advertise` a new `topic` and `Publish` data for other modulesa.


# First application (hello sky)
[PX4 dev ](https://dev.px4.io/en/apps/hello_sky.html)

## Minimal Application / module (without uORB)
- c file (examples/my_app folder)
```c
#include <px4_log.h>

__EXPORT int my_app_main(int argc, char *argv[]);

int my_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello my Sky!");
    return OK;
}
```
> The main method must named `<module_name>`_main

> `PX4_INFO` use px4 log facility

- CMakeLists.txt
```cmake
px4_add_module(
	MODULE examples__my_app
	MAIN my_app
	STACK_MAIN 2000
	SRCS
		my_app.c
	DEPENDS
	)
```
- Allocate module stack usage
- 

- Add to main cmake file `config_module_list`
  - SITL: cmake/configs/posix_sitl_default.cmake

- SITL
```
set(config_module_list
    ...
    examples/my_app
    # Orignal sample
    examples/px4_simple_app
    ...
)
```

### Test
Testing using SITL
- Run
```
make posix_sitl_default jmavsim
```

```
_____  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

pxh>
 ms4525_airspeed
  ms5525_airspeed
  my_app
  navigator

pxh> my_app
INFO  [my_app] Hello my Sky!

```

# Message / Publish / Subscribe Example
- message
- Advertise / Publish code
- Subscriber code

## Message
- Add new file to `msg` folder
- Add line to `CMakeLists.txt`

file: extctl_sp.msg
```
uint64 timestamp			# time since system start (microseconds)

bool run_pos_control
bool run_alt_control
bool run_yaw_control
float32 sp_yaw
float32 sp_x
float32 sp_y
float32 sp_z
float32 vel_sp_x
float32 vel_sp_y
float32 vel_sp_z
```

- Add to CMake file
```
set(msg_files
        battery_status.msg
        ...
        extctl_sp.msg
        ...)
```

> After compile crete header  and cpp file
> ./build/posix_sitl_default/uORB/topics/extctl_sp.h
./build/posix_sitl_default/msg/topics_sources/extctl_sp.cpp
./build/posix_sitl_default/msg/CMakeFiles/uorb_msgs.dir/topics_sources/extctl_sp.cpp.o
./build/posix_sitl_default/msg/tmp/sources/extctl_sp.cpp
./build/posix_sitl_default/msg/tmp/headers/extctl_sp.h

## Publisher
```C
#include <uORB/topics/extctl_sp.h>
#include <px4_log.h>
#include <px4.h>

__EXPORT int my_app_main(int argc, char *argv[]);

int my_app_main(int argc, char *argv[])
{
    //init struct
    struct extctl_sp_s _orb_sp = { 0 };
    //instance
    int _orb_sp_instance = -1;
    //
    orb_advert_t _orb_sp_topic = orb_advertise_multi(ORB_ID(extctl_sp), &_orb_sp, &_orb_sp_instance, ORB_PRIO_DEFAULT);
    //publish
    px4::Rate loop_rate(10);
    while (1){
        loop_rate.sleep();
        
    }
    int ret = orb_publish(ORB_ID(extctl_sp), _orb_sp_topic, &_orb_sp);

    return 0;
}
```

px4 examples location `Firmware/src/examples/`
## Reference 
- [PX4 Architectural Overview](https://dev.px4.io/en/concept/architecture.html)
- [PX4 Research Log [4] – A first look at PX4 architecture, example code, uORB and NSH script](https://uav-lab.org/2016/08/02/px4-research-log4-a-first-look-at-px4-architecture/)
- [UAV Gripper](https://www.slideshare.net/VinhNguyen154/harvard-hkust-2015-final-presentation-53041584)
- [ROS Drone](http://www.ros.org/news/2016/02/)
- [UAV-Lab](https://uav-lab.org/tutorials/software/px4-2/)
- [One day proficient in drones](https://www.zhihu.com/people/li-de-qiang-37/activities)
- [workshope](https://learn.subak.io/px4-workbook/)
- [subak.io workshope](https://subak.io/?page_id=34)
- [How  to customize pixhawk in your own project](http://nutshellking.com/articles/xue-xi-zong-jie/customize_Pixhawk/#1.3)