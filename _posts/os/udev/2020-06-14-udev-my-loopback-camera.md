---
layout: post
title: Write udev rule for loopback camera
categories: os
tags: [udev, v4l2loopback]
public: true
image: udev-linux.png
description: Write udev rule for loopback camera 
---

Write specific rule to assign symlink to loopback device

# LAB
- load and init two v4l loopback device
  - /dev/video3
  - /dev/video7
- Assign symlink
  - camera_in
  - camera_out


## Init multiple loopback

```
sudo modprobe v4l2loopback video_nr=3,7
```

## quey for info

```bash
sudo udevadm info -a /dev/video3
#
KERNEL=="video3"
SUBSYSTEM=="video4linux"
DRIVER==""
ATTR{format}==""
ATTR{index}=="0"
ATTR{buffers}=="8"
ATTR{name}=="Dummy video device (0x0000)"
ATTR{dev_debug}=="0"
ATTR{max_openers}=="10"

sudo udevadm info -a /dev/video7
#
KERNEL=="video7"
SUBSYSTEM=="video4linux"
DRIVER==""
ATTR{index}=="0"
ATTR{buffers}=="8"
ATTR{max_openers}=="10"
ATTR{dev_debug}=="0"
ATTR{name}=="Dummy video device (0x0001)"
ATTR{format}==""
```

### Write Rule
- Create file `99-myloop_camera.rules` at `/etc/udev/rules.d`


```
ACTION=="add", KERNEL=="video3", SUBSYSTEM=="video4linux", SYMLINK+="camera_in"
ACTION=="add", KERNEL=="video7", SUBSYSTEM=="video4linux", SYMLINK+="camera_out"
```


### Check
- load rules
- check for symlink
  
#### load rules

```bash
# load rules
sudo udevadm control --reload-rules
```

#### Check
```
ll /dev/camera_*

lrwxrwxrwx 1 root root 6 Jun 14 13:02 /dev/camera_in -> video3
lrwxrwxrwx 1 root root 6 Jun 14 13:02 /dev/camera_out -> video7
```

### Other way to rule declaration
- Using other attributes from info query 
```
ACTION=="add", SUBSYSTEM=="video4linux", ATTR{name}=="Dummy video device (0x0000)", SYMLINK+="camera_in"
ACTION=="add", SUBSYSTEM=="video4linux", ATTR{name}=="Dummy video device (0x0001)", SYMLINK+="camera_out"
```
