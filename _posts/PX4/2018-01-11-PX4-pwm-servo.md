---
layout: post
title: px4 pwm and servo control
categories: px4
tags: [px4, servo, pwm]
---

# pins
![](/images/2019-01-11-15-27-19.png)

1. RC: radio control receiver input
2. SB: S.Bus support
3. Main output
4. Aux output

![](/images/2019-01-11-15-29-36.png)


# Lab
- Connect pixhawk with usb
- Connect servo motor to Aux port 1
- Connect external power source (todo: what about shared ground)
- Send pwm command from Mavlink_shell
- Run QGC (open mavlink console)
- 
> Pixhawk need an external power source to pwm output to operate the motor

## PWM command
### Commands
- Main output: /dev/pwm_output0
- Aux output: /dev/pwm_output1

### info
Print current configuration of all channels
```
pwm info -d /dev/pwm_output1
```
![](/images/2019-01-12-10-34-58.png)

### test
Set output to a specific value
> Servo connect to AUX port 1  
> Channel rate 50hz

```
pwm test -d /dev/pwm_output1 -c 1 -p 2000
```
>  When we abort, servo return to it's 1500


## Lab (write pwm module)
- Move servo between -90 - 90 degree
