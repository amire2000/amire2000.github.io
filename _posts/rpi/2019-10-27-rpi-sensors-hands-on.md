---
layout: post
title: Interface Sensors With Raspberry Pi
categories: rpi
tags: [sensors, imu]
description: Connect imu and other sensors to RPi, using I2C and gpio
public: true
image: raspberry-pi.png
---

# Content
- Download and flash ubuntu server
- Init setup
- Connect IMU

## Step1 - download and flash
- Download ubuntu 18.04 [download](https://ubuntu.com/download/iot/raspberry-pi)
  - download ubuntu server
  - download 64 bit version
- Use [balenaEther](https://www.balena.io/etcher/) or other method to flash the image
- Default  user and password: `ubuntu`

## Step2 - connect with serial and boot
### Wiring

![](/images/2019-10-27-13-30-25.png)

- Connect 
  
![](/images/2019-10-27-13-28-08.png)

|  pin  |  Desc |
| ------|-------|
|  6    |   GND |
|  8    |   TX  |
|  10   |   RX  |

### Step3 - login using putty
- Check dmesg

![](/images/2019-10-27-13-33-36.png)

- putty


> Using `ubuntu` as default user and password

&nbsp;  
&nbsp;  
&nbsp;  
# I2C
- Connect IMU device

![](/images/2019-10-27-13-53-16.png)

## Install i2c utils
```
sudo apt update
sudo apt install python-smbus i2c-tools
```

## Scan

```
sudo i2cdetect -y 1
```

![](/images/2019-10-27-13-54-52.png)

## Add user to i2c group
- check /dev/i2c-i permission

```bash
ls -l /dev/i2c-1 
crw-rw---- 1 root i2c 89, 1 Jan 28  2018 /dev/i2c-1
```

- Add user to i2c group

```bash
sudo usermod -a -G i2c ubuntu
# check group members
id
uid=1000(ubuntu) gid=1000(ubuntu) groups=1000(ubuntu),4(adm),20(dialout),24(cdrom),25(floppy),27(sudo),29(audio),30(dip),44(video),46(plugdev),108(lxd),111(netdev)
# logout or use `su` to take permission affect
su user
password: 

id
uid=1000(ubuntu) gid=1000(ubuntu) groups=1000(ubuntu),4(adm),20(dialout),24(cdrom),25(floppy),27(sudo),29(audio),30(dip),44(video),46(plugdev),108(lxd),111(netdev),116(i2c)
```

&nbsp;  
&nbsp;  
&nbsp;  
# Demo: using MPU9250 sensor
## Read temperature
![](/images/2019-10-29-08-06-05.png)

```
read_i2c_block_data(i2c_addr,register,length,force=None)
```

```python
```

## Using python library

```
pip install FaBo9Axis_MPU9250
```


&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [MPU9250](http://43zrtwysvxb2gf29r5o0athu.wpengine.netdna-cdn.com/wp-content/uploads/2015/02/MPU-9250-Datasheet.pdf)
- [MPU-9250Register Map](https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)
- [smb api](https://buildmedia.readthedocs.org/media/pdf/smbus2/latest/smbus2.pdf)