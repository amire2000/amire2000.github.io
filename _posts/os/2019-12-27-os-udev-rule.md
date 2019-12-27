---
layout: post
title: Write udev rule for usb camera
categories: os
tags: [udev]
public: true
image: udev-linux.png
description: Write udev rule for usb camera 
---
On typical Linux-based systems, the `/dev` directory is used to store file-like device nodes which refer to certain devices in the system  
udev is the "new" way of managing `/dev` directories  
With udev rule we can control device name and run action when device add and remove at runtime

# Content
- udevadm
- Gather information
  - usb devices (camera, dvd)
- Write rule
  - SYMLINK
  - RUN
- Test

# udevadm
udev management tool 

- udevadm info `<options>`
- udevadm control
- udevadm trigger

&nbsp;  
&nbsp;  
&nbsp;  
# Gather information

Gather all the information needed to device identifer

identify usb device by 
- name
- id
  
## by id
```bash
lsusb
#
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 008: ID 045e:07b2 Microsoft Corp. 
Bus 003 Device 005: ID 0e8d:1806 MediaTek Inc. Samsung SE-208 Slim Portable DVD Writer
Bus 003 Device 003: ID 067b:2303 Prolific Technology, Inc. PL2303 Serial Port
Bus 003 Device 006: ID 045e:0779 Microsoft Corp. LifeCam HD-3000
```
- My usb camera connect at `Bus` 003 as `Device` 006
- Query device using `udevadm info`
> --attribute-walk:  Print all sysfs properties of the specified device that can be used in udev rules to match the specified device


```bash
udevadm info --attribute-walk /dev/bus/usb/003/006
#
looking at device '/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.2':
    KERNEL=="3-1.2"
    SUBSYSTEM=="usb"
    DRIVER=="usb"
    ATTR{authorized}=="1"
    ATTR{avoid_reset_quirk}=="0"
    ATTR{bConfigurationValue}=="1"
    ATTR{bDeviceClass}=="ef"
    ATTR{bDeviceProtocol}=="01"
    ATTR{bDeviceSubClass}=="02"
    ATTR{bMaxPacketSize0}=="64"
    ATTR{bMaxPower}=="500mA"
    ATTR{bNumConfigurations}=="1"
    ATTR{bNumInterfaces}==" 4"
    ATTR{bcdDevice}=="0106"
    ATTR{bmAttributes}=="80"
    ATTR{busnum}=="3"
    ATTR{configuration}==""
    ATTR{devnum}=="6"
    ATTR{devpath}=="1.2"
    ATTR{idProduct}=="0779"
    ATTR{idVendor}=="045e"
    ATTR{ltm_capable}=="no"
    ATTR{manufacturer}=="Microsoft"
    ATTR{maxchild}=="0"
    ATTR{quirks}=="0x0"
    ATTR{removable}=="unknown"
    ATTR{speed}=="480"
    ATTR{urbnum}=="5247"
    ATTR{version}==" 2.00"

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb3/3-1':
    KERNELS=="3-1"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
```

## by name
```bash
udevadm info -a -p $(udevadm info -q path -n /dev/video1)
```
- -a attribute walk
- -p path, the `/sys` path device to query
- -q query (nam, path, property)
- -n file name to query `/dev/xxx`

> The output like the result above

&nbsp;  
&nbsp;  
&nbsp;  
# Write rule
custom rules locate in `/etc/udev/rules.d`
The priority is defined by file name prefix number for example `60-my-usb-camera.rules`

Rule build from matching part and action part

## Add device symlink
```
KERNEL=="3-1.2", SUBSYSTEM=="usb", ATTR{idProduct}=="0779", ATTR{idVendor}=="045e", SYMLINK+="my_camera"
```

## Run
- Run script each time dvd portable usb device insert to usb port
> - Don't forget to `chmod` script as executable  
> - Set script full path
> - Disconnect and connect device to take affect `(ACTION=="add")`

```
ACTION=="add", KERNEL=="3-3", SUBSYSTEM=="usb", ATTR{idProduct}=="1806", ATTR{idVendor}=="0e8d", RUN+="/home/user/cd_in.sh"
```

- cd_in.sh (demo script)
  
```
#!/bin/sh
echo "cd in" > /tmp/cd
```
&nbsp;  
&nbsp;  
## Matching
### Device hierarchy
The Linux kernel actually represents devices in a tree-like structure
![](/images/2019-12-27-11-01-16.png)

- Matching device
  - KERNEL
  - SUBSYSTEM
  - DRIVER
  - ATTR
- Matching device parent
  - KERNELS
  - SUBSYSTEMS
  - DRIVERS
  - ATTRS

### String matching and substitution
> more string substitution and string matching in [Writing udev rule](http://www.reactivated.net/writing_udev_rules.html#strsubst)

## Acction
- NAME: node name to be created
- SYMLINK: Add name link to targeting node
- RUN: Add a program to the list of programs to be executed for a specific device


&nbsp;  
&nbsp;  
&nbsp;  
# Test
```
sudo udevadm control --reload && sudo udevadm trigger
```
## udev control
- Reload rules

```bash
sudo udevadm control --reload && sudo udevadm trigger
#
# --reload: Signal systemd-udevd to reload the rules files and other        databases (check man for more)
```

- Check rule result for `SYMLink` demo

```bash
ll /dev/my_camera
#
lrwxrwxrwx 1 root root 15 Dec 27 10:43 /dev/my_camera -> bus/usb/003/006
```
  
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [udev man](https://linux.die.net/man/7/udev)
- [udevadm man](https://linux.die.net/man/8/udevadm)
- [Writing udev rules](http://www.reactivated.net/writing_udev_rules.html)