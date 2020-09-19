---
layout: post
title: self-balancer IMU
categories: [mini, self_balancer]
tags: [imu]
public: true
description: self balancer robot imu
image: law.png
---

# Math reminder
## SOH, COH, TOA

![](/images/2020-09-18-06-39-25.png)


$$
sin\Theta = \frac{Opposite}{Hypotenuse}
$$

$$
cos\Theta = \frac{Adjacent}{Hypotenuse}
$$

$$
tan\Theta = \frac{Opposite}{Adjacent}
$$


&nbsp;  
&nbsp;  
&nbsp;  
# Accelerometer
- Accelerometer measure gravity the component of the acceleration due to gravity acting on each of the three axes 'g' 

![Pitch](/images/2020-09-18-15-41-50.png)
- Pitch demo


```
pitch = 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI
```
```
roll = 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI;
```

# Reference
- [
Nitin J. Sanket
Roboticist
](https://nitinjsanket.github.io/teaching.html)
- [Sensor Fusion for Orientation Estimation](https://www.mathworks.com/videos/matlab-and-simulink-robotics-arena-sensor-fusion-for-orientation-estimation-1541678310083.html)