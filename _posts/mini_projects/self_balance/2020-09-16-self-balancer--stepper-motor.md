---
layout: post
title: balancer-stepper-motor
categories: [mini, self_balancer]
tags: [stepping motors]
public: true
description: self balancer robot stepping motors
image: law.png
---

# Motor
![](/images/2020-09-16-17-40-02.png)


&nbsp;  
&nbsp;  
&nbsp;  

<hr>
# Driver
![](/images/2020-09-16-22-04-22.png)

### Microstep Selection Pins
`M0, M1, M2` use to set step size resolution
| M2  | M1  | M0  |           |
| --- | --- | --- | --------- |
| 0   | 0   | 0   | Full step |
| 0   | 0   | 1   | Half step |
| 0   | 1   | 0   | 1/4 step  |
| 0   | 1   | 1   | 1/8 step  |
| 1   | 0   | 0   | 1/16 step |
| 1   | 0   | 1   | 1/32 step |
| 1   | 1   | 1   | 1/32 step |


### Control input pins
- STEP (7): HIGH pulse , steps the motor by number of microsteps set by Microstep Selection Pins
- DIR (8): Spinning direction
  - HIGH: Clockwise
  - LOW: CounterClockwise

&nbsp;  
&nbsp;  
&nbsp;  
# Project wiring
| DVR8225      |            |
| ------------ | ---------- |
| 1 (EN)       |            |
| 2 (M0)       |            |
| 3 (M1)       | VCC        |
| 4 (M2)       |            |
| 5 (RST)      | VCC        |
| 6 (SLP)      | VCC        |
| 7 (STEP)     | Controller |
| 8 (DIR)      | Controller |
| 9 (GND)      | Common GND |
| 10 (FAULT)   |            |
| 11 (A2)      | Motor      |
| 12 (A1)      | Motor      |
| 13 (B1)      | Motor      |
| 14 (B2)      | Motor      |
| 15 (GND MOT) | POWER IN   |
| 16 (VMOT)    | POWER IN   |
&nbsp;  
&nbsp;  
&nbsp;  
<hr>
# Reference
- [Nema 14 , 35 BYGHW stepper motor](https://www.geeetech.com/nema-14-35-byghw-stepper-motor-p-909.html)
- [Geeetech StepStick DRV8825 Stepper Motor Driver](http://www.geeetech.com/wiki/index.php/DRV8825_Motor_Driver_Board)
- [Control Stepper Motor with DRV8825 Driver Module & Arduino](https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/)
- [Step Motor Basics](https://www.geckodrive.com/support/step-motor-basics/accuracy-and-resolution.html)