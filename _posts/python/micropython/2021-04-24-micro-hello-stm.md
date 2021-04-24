---
layout: post
title: MicroPython hello with STM NUCLEO
categories: python
tags: [micropython]
image: micropython.png
description: Setup and create dev environment for micro python on stm32 nucleo board using VSCode and dev tools, write GPIO and other basic demos
public: true
---

# LAB
- Install micropython firmware
- Write basic hello LED
- Demo code using internal SWITCH and LED
&nbsp;  
&nbsp;  
&nbsp;  
# Install
## Prerequisites
#### stlink-tools
stlink is an open source toolset to program and debug STM32 devices and boards manufactured by STMicroelectronics  
The STlink toolset includes:  
- st-info - a programmer and chip information tool
- st-flash - a flash manipulation tool
- more

```
sudo apt install stlink-tools
```

### dfu tool
> Found in `micropython/tools` folder
[micropython github tools folder](https://github.com/micropython/micropython/tree/master/tools)

&nbsp;  
&nbsp;  
&nbsp;  
## Firmware setup
- Download firmware for [NUCLEO_F446RE board ](https://micropython.org/download/stm32/)

> dfu file is the USB package for firmware upgrade on STM32 via USB

- Run
```bash
# erase 
st-flash erase
./tools/dfu.py --dump <firmware dfu file> 
st-flash write <firmware dfu file>.target0.image0.bin 0x08000000
st-flash write <firmware dfu file>.target0.image1.bin 0x08020000
```

### TODO
- Try `pydfu.py` tool 

```bash
# Search devices
./pydfu.py -l

# upload
./pydfu.py -u <firmware.dfu>
```

# Test
- console REPL
```bash
>>> import pyb
>>> pyb.LED(1).on()
>>> pyb.LED(1).off()
```

## `help()` command

![](/images/2021-04-24-08-30-43.png)

## Pin
- Read user push button 

```bash
>>> b = pyb.Pin('SW', pyb.Pin.IN)
>>> b.value()
1
# Push and read again
>>> b.value()
0
```

## Project
- Control user and led on board
&nbsp;  
&nbsp;  
- Pin Legend
![](/images/2021-04-24-09-26-17.png)
- Left side
![](/images/2021-04-24-09-26-55.png)
- Right side
![](/images/2021-04-24-09-27-22.png)
&nbsp;  
&nbsp;  
&nbsp;  

- main.py

```python
from machine import Pin
led = Pin('PA5', Pin.OUT_PP)            # Green LED on board
sw = Pin('PC13', Pin.IN, Pin.PULL_UP)   # USER SW on board
while True:
    led.value(0 == sw.value())          # invert
```
&nbsp;  
&nbsp;  
&nbsp;  
# References
- [Running micropython on stm32](https://www.carminenoviello.com/2015/06/03/running-micropyton-stm32nucleo-f4/)
- [Turtorial Examples for Nucleo-MicroPython](https://beta-notes.way-nifty.com/blog/2020/03/post-6ff8db.html)
- [STM32 Understanding GPIO Settings](https://electronics.stackexchange.com/questions/156930/stm32-understanding-gpio-settings)