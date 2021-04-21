---
layout: post
title: MicroPython hello with ESP8266/ESP32
categories: python
tags: [micropython]
image: micropython.png
description: Setup and create dev environment for micro python using VSCode and dev tools, write GPIO and other basic demos
public: true
---
# MicroPython
from official site: 
MicroPython is a lean and efficient implementation of the Python 3 programming language that includes a small subset of the Python standard library and is optimized to run on microcontrollers and in constrained environments. 

&nbsp;  
&nbsp;  
&nbsp;  
# Tools and VSCode
## Tools
### esptool
A command line utility to communicate with the ROM bootloader in Espressif ESP8266 & ESP32 microcontrollers.

Allows flashing firmware, reading back firmware, querying chip parameters, etc.

```
pip install esptools
```

## VSCode
- install Pymakr extension

### Config
Ctrl+Shift+P --> Pymakr > Global Settings

change `"auto_connect": false`
change `"address": "/dev/ttyUSB0"`

&nbsp;  
&nbsp;  
&nbsp;  
# Firmware
- From MicroPython official site download [8266](https://micropython.org/download/esp8266/) firmware

- Erase flash
```
esptool.py --port /dev/ttyUSB0 erase_flash
```
- Upload firmware
```
esptool.py --port /dev/ttyUSB0 --baud 460800 write_flash --flash_size=detect -fm dio 0 esp8266-20170108-v1.8.7.bin
```

[More details](http://docs.micropython.org/en/latest/esp8266/tutorial/intro.html#deploying-the-firmware)

&nbsp;  
&nbsp;  
&nbsp;  
# Code

## LED Blink (ESP8266)

![](/images/2021-04-21-06-32-38.png)

![](/images/2021-04-21-06-33-42.png)

|        | LED A | LED B  |
| ------ | ----- | ------ |
| Pin    | 2     | 16     |
| Name   | GPIO2 | GPIO16 |
| sketch | D4    | D0     |

> Both LED work negative logic
> LED OFF: when pin is HIGH (on, value 1/True) 
> LED ON: when pin is LOW (off, value 0/False) 

#### main.py

```python
from machine import Pin
import time
p2 = Pin(2, Pin.OUT)
p16 = Pin(16, Pin.OUT)
while True:
    p2.on()
    p16.off()
    time.sleep(1)
    p2.value(False)
    p16.value(1)
    time.sleep(1)
```

> Use Pymakr to `download / upload` project files

&nbsp;  
&nbsp;  
&nbsp;  
# MicroPython libraries
[Read more](http://docs.micropython.org/en/latest/library/index.html)

```
help('modules')
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference

- [MicroPython](https://micropython.org/)
- [MicroPython docs](http://docs.micropython.org/en/latest/index.html)


![](/images/2021-04-21-06-42-17.png)
