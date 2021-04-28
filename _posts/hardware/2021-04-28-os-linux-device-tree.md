---
layout: post
title: Linux Device Tree
categories: hw
tags: [device tree, dtc, dts]
image: gpio.png
description: Linux Device Tree
public: true
---

- DTS: Device Tree Source
- DTC: Device Tree Compiler
- DTB: Device Tree Blob
- dtsi: DTS include file
- dts overlay: override properties values from include files

- Device like I2C, SPI, GPIO don't have capability to announce their existing to the OS.
- The kernel no longer contains the description of the hardware, it is locate in a separate binary  (DTB)
- The DTB is data structure file,that describe a machine hardware configuration

Check Device Tree loaded by the kernel
- Each property is a file
```
ls -l /sys/firmware/devicetree/base/
```


## Modify Device Tree at runtime
- U-Boot
- Device Tree Overlays


# References
- [Device Tree: hardware description for everybody !](https://youtu.be/Nz6aBffv-Ek)
- [How do I make a device tree overlay for just a single GPIO?](https://raspberrypi.stackexchange.com/questions/43825/how-do-i-make-a-device-tree-overlay-for-just-a-single-gpio)
- [GPIO and Device Tree](http://derekmolloy.ie/gpios-on-the-beaglebone-black-using-device-tree-overlays/)