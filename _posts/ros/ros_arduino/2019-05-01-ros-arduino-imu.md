---
layout: post
title: ROS Arduino and IMU Sensor
categories: ROS
tags: [ros, arduino, imu]
image: gyro.png
---
# MPU9250

## Accelerometers
Accelerometers are devices that are capable of measuring the acceleration they experience relative to free-fall


# Arduino interface
- 

## Config / Set 
###  Gyroscope Configuration
gyroscope full-scale range of ±250, ±500, ±1000, and ±2000°/sec (dps)
![](/images/2019-05-03-14-04-58.png)

###  Accelerometer Configuration
accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g
![](/images/2019-05-03-14-07-33.png)

![](/images/2019-05-04-00-44-19.png)
- Oriented by the dot (on chip)
![](/images/2019-05-04-01-08-13.png)


### Magnetometer Configuration
magnetometer full-scale range of ±4800μT
> Z positive are down
![](/images/2019-05-04-01-09-02.png)
## Read sensor data
Read data start from address `0x3b`
- Accelerator
- Temperature
- Gyro
  
- Register data  
![](/images/2019-05-03-09-04-13.png)

## Calc Pitch and Roll
$$
pitch = arctan(\frac{Gy}{^{\sqrt{G{x}^{2}}+G{z}^{2}}})
$$
$$
roll=arctan(\frac{-G_{x}}{G_{z}})
$$

### Find i2c address

```
#include <Wire.h>
void setup() {
  Serial.begin (9600); 
  while (!Serial) // Waiting for serial connection
    {
    }

  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}  // end of setup

void loop() {}
```

![](/images/2019-05-03-08-58-01.png)

## Reference
- [Beginner guide to IMU](http://students.iitk.ac.in/roboclub/2017/12/21/Beginners-Guide-to-IMU.html)
- [i2c scanner](https://roboindia.com/tutorials/i2c-scanner)
- [MPU-9250 Datasheet](https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [MPU-9250 register map](https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf)
- [ROS, IMU and an Arduino](https://atadiat.com/en/e-ros-imu-and-arduino-how-to-send-to-ros/)
- [ROS camera and IMU synchronization](http://grauonline.de/wordpress/?page_id=1951)