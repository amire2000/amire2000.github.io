---
layout: post
title: Arduino RPi3 Serial
categories: Arduino
tags: [arduino, serial, rpi3]
---

![](/images/2019-01-21-20-33-40.png)

![](/images/2019-01-21-20-42-18.png)
## Connection
- Arduino(Rx) -> RPi(Tx)
- Arduino(Tx) -> RPi(Rx) with power divider
- use voltage regulator
    -  Arduino serial  pin held 5v
    -  RPi  use 3.3v

$$
V_{out} = V_{in}*\frac{R_{2}}{R_{1}+R_{2}}
$$

$$
3.3 = 5 * \frac{3300}{1600 + 3300}
$$

## RPi3 GPIO  Serial
>/dev/ttyAMA0 -> Bluetooth  
/dev/ttyS0 -> GPIO serial port

### Enabling
GPIO serial are disabled by default
to enable edit `config.txt`

```bash
sudo nano /boot/config.txt

# add
enable_uart=1
```

### Disabling the console
- disabled serial getty
```bash
sudo systemctl stop serial-getty@ttyS0.service
$ sudo systemctl disable serial-getty@ttyS0.service
```
- You also need to remove the console from the `cmdline.txt`

```bash
sudo nano /boot/cmdline.txt
```
remove the part: `console=serial0,115200` save and reboot for changes to take effect.

##  Arduino 
- upload demo code
```c
byte number = 0;
const int8_t LED =  13;
void setup()
{
    pinMode(LED, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    if (Serial.available())
    {
        number = Serial.read();
        Serial.print("character received: ");
        Serial.println(number, DEC);
        if (number  == char('a')){
            digitalWrite(LED, HIGH);
        }
        else{
            digitalWrite(LED, LOW);
        }
    }
}
```

## RPi
- Run  minicom or miniterm.py
- Type: Arduino return char ASCII code

```bash
minicom -b 9600 -o -D /dev/ttyS0
# or
miniterm.py /dev/ttyS0
```

##  Python
- Install pySerial
```
sudo apt-get install python-serial
```

- Demo
```python
import serial                                 ser = serial.Serial("/dev/ttyS0", 9600, timeout=1)                                    ser.open() 
ser.write(str.encode('a'))
r=ser.readline()
print (r)                                     #b'character recieved: 97\r\n'
ser.close()


```
## Reference
- [Configuring The GPIO Serial Port On Raspbian Jessie and Stretch Including Pi 3](https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3/)
- [RASPBERRY PI AND ARDUINO CONNECTED OVER SERIAL GPIO](https://oscarliang.com/raspberry-pi-and-arduino-connected-serial-gpio/)