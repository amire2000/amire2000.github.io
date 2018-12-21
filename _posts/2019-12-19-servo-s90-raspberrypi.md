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

### Pi PWM mode
- MarkSpace mode: Traditional (PWM-MS)
- Balance mode: Default (PWM-BAL)

Rpi bask frequency are `19.2MHz`
- GPIO PWMC: Clock divider
- GPIO PWMR: Period range

#### Calc for MarkSpace mode
{% raw %}
- $Tq: 100\mu s$
- pwmClock
$$
divisor = 19200000[Hz] * 0.0001[s] = 1920
$$
- pwmRange
$$
PWM Freq = \frac{19.2MHz}{PWMC * PWMR}
$$

$$
50 = \frac{10000}{range}
$$
OR
$$
range=\frac{T}{Tq}=\frac{20[ms]}{0.1[ms]}=200
$$
- Calc values
$$
value(1.5ms)=\frac{T_H}{Tq}=\frac{1.5[ms]}{0.1[ms]}=15
$$
$$
value(2ms)=\frac{T_H}{Tq}=\frac{2[ms]}{0.1[ms]}=20
$$
{% endraw %}
### Code example (Tq as $10\mu s$)
- Clock divsor: 192
- Range: 2000
  
```python
#
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

delay_period = 0.01

while True:
    for pulse in range(100, 200, 1):
        wiringpi.pwmWrite(18, pulse)
        print pulse
        time.sleep(delay_period)
    for pulse in range(200, 100, -1):
        wiringpi.pwmWrite(18, pulse)
        print pulse
        time.sleep(delay_period)
```

![](/images/2018-12-20-08-11-14.png)

## Reference 
- [SG90 Data sheet](http://www.towerpro.com.tw/product/sg90-7/)
- [How Do Servo Motors Work?](https://www.jameco.com/jameco/workshop/howitworks/how-servo-motors-work.html)

> GPIO 12 , GPIO 13 are hardware PWM outputs.

> Dead band width:  
> That's the amount the signal is allowed to change without affecting the output