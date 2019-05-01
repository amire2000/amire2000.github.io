---
layout: post
title: RUN ROS on Odroid
categories: ros
tags: [ros, odroid]
image: odroid_xu4.jpg
description: Install ROS Melodic on ODROID board and connect it to Arduino Mega
public: true
---

### Objective
- Download and flash ubuntu image
- Install ROS melodic
- Install ROS arduino packages
- Run blink Demo

![image alt img](/images/odroid.png) 


### Hardware
- Odroid XU4
  - Odroid power supply 5v 4A
- Arduino Mega
  - Usb C-Type to D-Type (Odroid arduino connection)
  - Arduino power supply
### Software
- Arduino IDE (Workstation)
- [Etcher](https://www.balena.io/etcher/) Flash OS images to SD cards (Workstation)
- ROS Melodic (Workstation, odroid)

## Install instructions

- Download img [18.04 4.14  minimal](https://odroid.in/?directory=.%2Fubuntu_18.04lts%2FXU3_XU4_MC1_HC1_HC2%2F)
- Using etcher to flash the image
- Insert uSD into
  - switch `Boot mode selector` to uSD
  - Power on (alive led blinkin blue)
- ssh 
  - user: root
  - pass: odroid
  - shell has warning
    ```bash
    bash: warning: setlocale: LC_ALL: cannot change locale (en_US.UTF-8)
    ```
- Run `locale-gen en_US.UTF-8` to fix the warning
- Run `apt update`

## Install ROS Melodic
- Install `ros-melodic-ros-base`
- Follow instruction from 
[Ubuntu install of ROS Melodic ](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Odroid prepare 
- Add user
```bash
useradd -ms /bin/bash user
# add user to sudoers
echo "user   ALL=(ALL:ALL) ALL" >> /etc/sudoers
# set password
echo "user:user" | chpasswd
# Add user to dialup
usermod -a -G dialout user
# Add .profile 
echo "source .profile >> .bashrc"
# Add ROS into .profile
echo "source /opt/ros/melodic/setup.bash" > /home/user/.profile
```
- -m: create /home directory
- -s: set shell

> Add `ROS setup`  into user .profile (tmux in mind: tmux not read .bashrc)

## Workstation
```bash
sudo apt install ros-melodic-rosserial-arduino
sudo apt install ros-melodic-rosserial
```
### Install ros_lib
- Build ros_lib in arduino library folder
```bash
cd ~/Arduino//libraries
#run
rosrun rosserial_arduino make_libraries.py .
```

## Arduino 
- Download [arduino IDE](https://www.arduino.cc/en/Main/Software)
- Open arduino ide after build the `ros_lib`
- Upload blink sketch from `Examples->ros_lib->blink`

## Check from Workstation
- Terminal 1
```bash
roscore
```

- Terminal 2
```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

- Terminal 1
```bash
#toggle on
rostopic pub -1 /toggle_led std_msgs/Empty "{}"
#toggle off
rostopic pub -1 /toggle_led std_msgs/Empty "{}"
```

## Move to ODROID
> Tip: install tmux

> Tip: Don't forget to add user to dialout group
### install packages
```bash
sudo apt install ros-melodic-rosserial
```

### tmux
- pane 1
```
roscore
```

- pane2
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

- pane3
```bash
#view topics
rostopic list
/rosout
/rosout_agg
/toggle_led

# send message to toggle LED on
rostopic pub -1 /toggle_led std_msgs/Empty "{}"
# send message to toggle LED off
rostopic pub -1 /toggle_led std_msgs/Empty "{}"
```

![](/images/ros_odroid_arduino.jpeg)

# Reference
- [Odroid wiki](https://wiki.odroid.com/odroid-xu4/hardware/hardware)
- [Arduino IDE Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)