---
layout: post
title: NVIDIA jetson custom image
categories: nvidia
tags: [nvidia, jetson, image, toolchain]
public: true
image:
description:
---
- L4T: Linux for Tegra
- BSP: board support package
- JetPack: 32.3.1



# Jetson
- Jetson TX2(P3310-1000)
# Toolchain
![](/images/2020-03-25-20-28-40.png)
- Download `GCC 7.3.1 for 64 bit BSP and Kernel`
  - It's download `gcc-linaro-7.3.1`
- Extract
  
# Kernel

# Flash
```
sudo ./flash.sh jetson-tx2 mmcblk0p1
```

![](/images/2020-03-25-21-03-37.png)

![](/images/2020-03-25-21-05-44.png)

![](/images/2020-03-25-21-06-19.png)


## Avidea
[firmware](https://auvidea.eu/firmware/)
```
sudo cp -r Linux_for_Tegra/* ~/nvidia/nvidia_sdk/JetPack_4.3_Linux_P3310/Linux_for_Tegra/
sudo ./apply_binaries.sh
```
## Check for cuda
/usr/local/cuda/samples/1_Utilities/deviceQuery
make TARGET_ARCH=aarch64
scp and check


## OpenCV
- linaro
- bin/aarch64-linux-gnu-gcc
- bin/aarch64-linux-gnu-g++



# Reference
- [Compiling Jetson TX1/TX2 source code](https://developer.ridgerun.com/wiki/index.php?title=Compiling_Jetson_TX1/TX2_source_code)
- [NVIDIA Jetson Linux Driver Package Developer Guide](https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3231/index.html#page/Tegra%2520Linux%2520Driver%2520Package%2520Development%2520Guide%2Fquick_start.html%23wwpID0E0WD0HA)
- [Flashing and Booting the Target Device](https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3231/index.html#page/Tegra%2520Linux%2520Driver%2520Package%2520Development%2520Guide%2Fflashing.html%23)
- [nvidia deb to download](http://169.44.201.108:7002/jetpacks/4.3/)
- [Jetson Download Center](https://developer.nvidia.com/embedded/downloads)
- [Yocto](https://hub.mender.io/t/nvidia-tegra-jetson-tx2/123)
## core
- [A Guide to Ubuntu Core and Snaps](https://hackernoon.com/a-guide-to-ubuntu-core-and-snaps-5b3b67b0188d)
## rootfs
- [Jetson/TX1 Sample Root Filesystem](https://elinux.org/Jetson/TX1_Sample_Root_Filesystem)
- [Building Ubuntu rootfs for ARM](https://gnu-linux.org/building-ubuntu-rootfs-for-arm.html)
- [Sample Filesystems (nVidia Tegra TX2)](https://github.com/Abaco-Systems/jetson-tx2-sample-filesystems)
- [ubuntu base](http://cdimage.ubuntu.com/ubuntu-base/releases/18.04.3/release/)