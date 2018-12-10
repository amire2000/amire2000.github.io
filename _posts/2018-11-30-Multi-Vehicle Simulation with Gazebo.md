---
layout: post
title: Multi-Vehicle Simulation with Gazebo
categories: px4
tags: [sitl, gazebo, px4]
---
### Gazebo env. variables
- GAZEBO_MODEL_PATH: where gazebo search for modules
- GAZEBO_PLUGIN_PATH: where gazebo search for plugins

### PX4 sitl folders
- models: Firmware/Tools/sitl_gazebo/models
- worlds: Firmware/Tools/sitl_gazebo/worlds
- plugins: Firmware/build/posix_sitl_default/build_gazebo/
- scripts: Firmware/Tools 

### Build world and clone model
- use empty world
- clone iris model

#### world
~~~bash
cd Firmware/Tools/sitl_gazebo/worlds
cp iris.world iris_multi.world
~~~

- Add / Include another model
- Changed model position

~~~
<!-- iris     -->
    <include>
      <uri>model://iris</uri>
      <pose>1.01 0.98 0.83 0 0 1.14</pose>
    </include>

    <!-- iris1 -->
    <include>
      <uri>model://iris1</uri>
      <pose>2.01 0.98 0.83 0 0 1.14</pose>
    </include>
~~~
#### Clone model
~~~
cd Firmware/Tools/sitl_gazebo/models
cp iris iris1
~~~
- Changed model name attribute
- Changed sitl port

~~~xml
<sdf version='1.6'>
  <model name='iris'>

<sdf version='1.6'>
  <model name='iris1'>
~~~

~~~xml
<mavlink_addr>INADDR_ANY</mavlink_addr>
      <mavlink_udp_port>14560</mavlink_udp_port>

<mavlink_addr>INADDR_ANY</mavlink_addr>
      <mavlink_udp_port>14561</mavlink_udp_port>
~~~

### Run 
- Terminal 1
  
~~~bash
Tools/sitl_multiple_run.sh

killing running instances
starting instance 0 in /home/user/px4/Firmware/build/posix_sitl_default/instance_0
starting instance 1 in /home/user/px4/Firmware/build/posix_sitl_default/instance_1


# output example
# cat /home/user/px4/Firmware/build/posix_sitl_default/instance_0/out.log
______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [Unknown] Calling startup script: /bin/sh etc/init.d-posix/rcS 0
INFO  [dataman] Unknown restart, data manager file './dataman' size is 11405132 bytes
INFO  [simulator] Waiting for initial data on UDP port 14560. Please start the flight simulator to proceed..
INFO  [init] Mixer: etc/mixers/quad_w.main.mix on /dev/pwm_output0

~~~

- Terminal 2 (gazebo)
  
~~~bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/user/px4/Firmware/Tools/sitl_gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/user/px4/Firmware/build/posix_sitl_default/build_gazebo/

gazebo --verbose Tools/sitl_gazebo/worlds/iris_multi.world
~~~

![](/images/2018-11-30-18-21-53.png)

## Run QGroundcontrol
- qgroundcontrol detect the to vehicle
> The SITL wait for connection from gazebo (mavlink plugin) and then continue to run 

![](/images/2018-11-30-22-11-03.png)

![](/images/2018-11-30-22-14-39.png)

![](/images/2018-11-30-22-13-59.png)

- Run takeoff from qgroundcontrol

![](/images/2018-11-30-22-16-37.png)


### SITL Ports
![](/images/2018-11-30-22-30-25.png)

- Each SITL listen in different port

## Resource
- [px4](https://dev.px4.io/en/simulation/multi-vehicle-simulation.html)


