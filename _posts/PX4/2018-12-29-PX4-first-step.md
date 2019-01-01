---
layout: post
title: PX4 first step
categories: px4
tags: [px4, pixhawk]
---
# Install px4
- First connection to pixhawk
  - The default stack `Ardupilot`
![](/images/2018-12-31-21-51-15.png)
-  check 
```
usb 3-4: New USB device found, idVendor=26ac, idProduct=0011
usb 3-4: New USB device strings: Mfr=1, Product=2, SerialNumber=3
usb 3-4: Product: PX4 FMU v2.x
usb 3-4: Manufacturer: 3D Robotics
usb 3-4: SerialNumber: 0
cdc_acm 3-4:1.0: ttyACM0: USB ACM device
```

## Firmware upgrade from QGC (px4)
- Disconnect and connect agian to get the selection choice
![](/images/2018-12-31-22-00-00.png)

- After process and calibration
![](/images/2019-01-01-07-38-51.png)

- Check version  (QGC)  
![](/images/2019-01-01-10-06-27.png)

- Check version from mavlink shell (get shell)

```
nsh> ver all
HW arch: PX4FMU_V2
HW type: V2
HW version: 0x00090008
HW revision: 0x00000000
FW git-hash: 82aa24adfca29321cfd1209e287eab6c2b16780e
FW version: Release 1.8.1 (17302015)
OS: NuttX
OS version: Release 7.22.0 (118882559)
OS git-hash: 63775322bf25adb406594f8e610122fe0cef2f7a
Build datetime: Oct 20 2018 00:16:40
Build uri: BUILD_URI
Toolchain: GNU GCC, 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
MFGUID: 343633323535510500320043
MCU: STM32F42x, rev. 3
UID: 320043:35355105:34363332 
```

# Custom Firmware with app
## Setup system to allow upload 
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager
```

## Without docker

## Use docker as Toolchain
- [px4 build](https://dev.px4.io/en/setup/building_px4.html)
- [px4 docker](https://dev.px4.io/en/test_and_ci/docker.html)

### Docker container list
- px4-dev-base: 	Base setup common to all containers  
    - px4-dev-nuttx:	NuttX toolchain

### Usage
- Get docker image
- Compile with image
- Upload image to pixhawk

#### Get docker image
```
docker pull px4io/px4-dev-nuttx
```

#### Compile 
View all targets `make list_config_targets`
- pixhawk: make px4fmu-v2_default
- pixhawk2: make px4fmu-v3_default

- use help scrips `./Tools/docker_run.sh make px4fmu-v2_default`
> Use the matching Firmware TAG

> upload action hang

### Resolve or not
```
export SRC_DIR=`pwd`
docker run -it --rm --name=px4-dev-nuttx \
--privileged \
--env=LOCAL_USER_ID="$(id -u)" \
--volume=/dev/bus/usb:/dev/bus/usb \
--volume=${SRC_DIR}:${SRC_DIR}:rw \
px4io/px4-dev-nuttx:2018-09-11 /bin/bash

```

## Update new FW with QGC
![](/images/2019-01-01-11-41-44.png)

![](/images/2019-01-01-11-46-07.png)


## Get shell
[px4](http://dev.px4.io/en/debug/system_console.html)
- Mavlink shell
- Pixhawk serial
- 
> Close QGC


### Mavlink shell
- From Firmware Tool sub folder `mavlink_shell.py`
  - > hit enter to get the prompt `Connecting to MAVLINK`
```
Using port /dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00
Connecting to MAVLINK...

nsh> 
```
## reference
- [stmoon](https://stmoon.github.io/pixahwke-modyul-cugahaebogi-hello-world/)
- [how to customized pixhawk in your onw project](http://nutshellking.com/articles/xue-xi-zong-jie/customize_Pixhawk/)
- [Development Environment on Linux
](http://dev.px4.io/en/setup/dev_env_linux.html)
-[A Setup for Multi-UAV Hardware-in-the-Loop
Simulations](http://www.kyb.tuebingen.mpg.de/fileadmin/user_upload/files/publications/2015/RED-UAS-2015-Odelga.pdf)