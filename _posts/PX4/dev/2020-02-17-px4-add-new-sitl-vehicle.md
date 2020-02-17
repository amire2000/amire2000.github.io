---
layout: post
title: Add new SITL vehicle
categories: px4
tags: [sitl]
public: true
description: Add new vehicle included gazebo module
image: px4-inside.png
---

# Steps

- Custom module
- Custom world
- Add start script 
- Add vehicle mixer
- Modify cmake
&nbsp;  
&nbsp;  
# summary
- airframe: my_rover
- airframe id: 10601
- module location: `Tools/sitl_gazebo/models`
- world location: `Tools/sitl_gazebo/worlds`
- sitl startup file: `ROMFS/px4fmu_common/init.d-posix`
- sitl mixer file location: `etc/mixers-sitl/`
- sitl cmake location: `platforms/posix/cmake/`

## Custom module
- Create new folder `my_rover` under `Tools/sitl_gazebo/models`

## Custom world
- Add new world file `my_rover.world` under `Tools/sitl_gazebo/worlds`

## Startup script
- Add file with unique airframe number under `ROMFS/px4fmu_common/init.d-posix`


## Add mixer file
Add mixer file `my_rover_sitl.main.mix` to `etc/mixers-sitl/`

## Modify cmake
- Add airframe name (my_rover) to `platforms/posix/cmake/sitl_target.cmake` `set(module` section

```
set(models none shell
	if750a iris iris_opt_flow iris_vision iris_rplidar iris_irlock iris_obs_avoid solo typhoon_h480
	plane
	standard_vtol tailsitter tiltrotor
	hippocampus rover my_rover)
```

## Run
```
make px4_sitl gazebo_my_rover
```

![](/images/2020-02-17-07-04-19.png)
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Create custom moddedl for sitl](https://discuss.px4.io/t/create-custom-model-for-sitl/6700/3)
- [Adding new airframe](https://dev.px4.io/v1.9.0/en/airframes/adding_a_new_frame.html)