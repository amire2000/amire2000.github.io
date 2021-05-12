---
layout: post
title: Linux ubuntu control GPIO with new libgpiod library
categories: hw
tags: [gpio]
image: gpio.png
description: Control GPIO on jetson nano with new library and tools from libgpiod (ubuntu 20.04)
public: true
---

# LAB

- OS: ubuntu 20.04
- HW: jetson nano

```bash
# install
sudo apt install gpiod
# install python bindings
sudo apt install python3-libgpiod
```

## Permission

```
sudo groupadd gpio
sudo usermod -G gpio <user>
```

### udev

- create file in `/etc/udev/rules.d` for example `60-gpio.rule`

```
SUBSYSTEM=="gpio", KERNEL=="gpiochip[0-4]", GROUP="gpio", MODE="0660"
```

```
ll /dev/gpiochip*
crw------- 1 root root 254, 0 Apr  1  2020 /dev/gpiochip0
crw------- 1 root root 254, 1 Apr  1  2020 /dev/gpiochip1
```

```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

```
ll /dev/gpiochip*
crw-rw---- 1 root gpio 254, 0 May  1 17:23 /dev/gpiochip0
crw-rw---- 1 root gpio 254, 1 May  1 17:23 /dev/gpiochip1
```

## tools

- gpiodetect: list all gpiochips present on the system, their names, labels and number of GPIO lines

```
gpiodetect
gpiochip0 [tegra-gpio] (256 lines)
gpiochip1 [max77620-gpio] (8 lines)
```

| pin gpio | sysfs   | line |
| -------- | ------- | ---- |
| 7        | gpio216 | 216  |
| 11       | gpio50  | 50   |
| 15       | gpio194 | 194  |
| 33       | gpio38  | 38   |


```
gpioset gpiochip0 38=1
gpioset gpiochip0 38=0

gpioget gpiochip0 38

```