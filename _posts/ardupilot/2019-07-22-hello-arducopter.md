---
layout: post
title: ArduCopter SITL and Gazebo hello world
categories: apm
tags: [arducopter, sitl, gazebo]
---

# Build environment
```bash
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
```

## Additional packages and settings
```bash
# version
scripts/install-prereqs-ubuntu.sh

# version
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
 
> Note: Don't forget to source .bashrc

# SITL
Run ArduCopter

```bash
cd ardupilot/ArduCopter

# First run wipe virtual EEPROM (save params)
sim_vehicle.py -w

# Run SITL with map and console
# ardupilot/ArduCopter/Tools/autotest/sim_vehicle.py
sim_vehicle.py --console --map

```
![](/images/2019-07-23-21-11-19.png)



# Reference
- [Setting up the Build Environment](http://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
- [Copter SITL/MAVProxy Tutorial](http://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html)