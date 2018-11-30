---
layout: post
title: Multi-Vehicle Simulation with Gazebo
categories: px4
tags: [sitl, gazebo, px4]
---

# Multi-Vehicle Simulation with Gazebo

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
```
cd Firmware/Tools/sitl_gazebo/worlds
cp iris.world iris_multi.world


```
- Add / Include another model
- Changed model position
```
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
```
#### Clone model
```
cd Firmware/Tools/sitl_gazebo/models
cp iris iris1
```
- Changed model name attribute
- Changed sitl port

```xml
<sdf version='1.6'>
  <model name='iris'>

<sdf version='1.6'>
  <model name='iris1'>
```

```xml
<mavlink_addr>INADDR_ANY</mavlink_addr>
      <mavlink_udp_port>14560</mavlink_udp_port>

<mavlink_addr>INADDR_ANY</mavlink_addr>
      <mavlink_udp_port>14561</mavlink_udp_port>
```

### Run 
- Terminal 1
```
Tools/sitl_multiple_run.sh

killing running instances
starting instance 0 in /home/user/px4/Firmware/build/posix_sitl_default/instance_0
starting instance 1 in /home/user/px4/Firmware/build/posix_sitl_default/instance_1
```

- Terminal 2 (gazebo)
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/user/px4/Firmware/Tools/sitl_gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/user/px4/Firmware/build/posix_sitl_default/build_gazebo/

gazebo --verbose Tools/sitl_gazebo/worlds/iris_multi.world
```

![](/images/2018-11-30-18-21-53.png)

## Resource
- [px4](https://dev.px4.io/en/simulation/multi-vehicle-simulation.html)


