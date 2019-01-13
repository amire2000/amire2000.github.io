---
layout: post
title: PX4 compilers and versions
categories: px4
tags: [px4, docker, toolchains]
---

## Toolchains
- Download from [developer.arm.com](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)

|  toolchain   |  version   |
| --- | --- |
|  gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2   |  v1.8.1   |


# Docker
Run toolchain from docker
```
docker run -it --privileged \
    --rm \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v `pwd`:/home/user/Firmware:rw \
    --name=px4_compiler \
    px4io/px4-dev-nuttx:2018-09-11 \
    /bin/bash
```
- Check compiler version
```
user@2aea0b3a8d33:~/Firmware$ arm-none-eabi-gcc --version
arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
```
# Reference
- [Ubuntu toolchain](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#nuttx-based-hardware)