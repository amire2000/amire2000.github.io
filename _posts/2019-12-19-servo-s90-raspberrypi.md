---
layout: post
title: servo motor and raspberryPi
categories: hardware
tags: [servo, raspberry]
---
This post show how to control servo motor using raspberry and python.
The motor shaft can be positioned to specific angular positions by sending the servo a coded signal. As long as the coded signal exists on the input line, the servo will maintain the angular position of the shaft.  
Servo are controlled by sending PWM signal, usually servo turn 90 degree in either direction (180 total)

![](/images/2018-12-20-07-49-48.png)




## Servo motor sg90
- Operation voltage: 4.8v
- Dead band width: 1us
- Operating speed: 0.1sec/60degree(4.8v)


![](/images/2018-12-20-00-49-24.png)
![](/images/2018-12-20-07-52-26.png)  

![](/images/2018-12-20-00-50-09.png)


## Wire and control from Pi
- Install library to control gpio
```
sudo apt-get install -y wiringpi
```
- Install python wrapper for this library
```
sudo pip install wiringpi
```

### Code example
```python
import time
import wiringpi
 
# For GPIO pin numbering'
wiringpi.wiringPiSetupGpio()
 
# set #18 to be a PWM output
wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)
 
# set the PWM mode to milliseconds stype
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
 
# divide down clock
wiringpi.pwmSetClock(192)
wiringpi.pwmSetRange(2000)
```

![](/images/2018-12-20-08-11-14.png)

## Reference 
- [SG90 Data sheet](http://www.towerpro.com.tw/product/sg90-7/)
- [How Do Servo Motors Work?](https://www.jameco.com/jameco/workshop/howitworks/how-servo-motors-work.html)