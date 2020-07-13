---
layout: post
title: Connect TFMini LIDAR sensor to jetson NANO (uart)
categories: hw
tags: [sensor]
image: 
description: Connect TFMini LIDAR uart to jetson NANO and other 
public: true
---

# TFMini
Fmini is based on TOF, namely, Time of Flight principle. To be specific, the product transmits modulation wave of near infrared rayon a periodic basis,which wave will reflect after contacting object. The product obtains time of flight by measuring round-trip phase difference and then calculates relative range between the product and the detection object

## Key characteristics
- Range (indoor): 0.3-12m
- accuracy: +-4cm (0.3-6m)
- check manual for more

## Wiring
![](/images/2020-07-12-17-28-40.png)

## Serial protocol
- baud 115200
- Data bit: 8
- Stop bit: 1
- Parity: None
### Serial data
![](/images/2020-07-12-18-09-34.png)
- Byte 0-1: 0x59, frame header, same for each frame
- Byte 2: Dist_L distance value lower by 8 bits
- Byte 3: Dist_L distance value higher by 8 bits
- Byte 4: Strength_L low 8 bits
- Byte 5: Strength_L high 8 bits
- Byte 6: mode: check manual
- Byte 7: 00 by default
- Byte 8: CheckSum is thelow 8 bits of the cumulative sum of the numbers of the first 8 bytes
# Jetson NANO
- Disable getty service, 
- Add user to dialout group
  
```bash
# disable getty service
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty 
# add user to dialout group
sudo usermod -a -G dialout $USER
```

# j41
> nano uart operate at 3.3v

![](/images/2020-07-12-17-21-03.png)

# Wiring
| TF  | Nano    |
| --- | ------- |
| 5V  | 4       |
| GND | 6       |
| RX  | 8 (tx)  |
| TX  | 10 (rx) |

&nbsp;  
&nbsp;  
# Python sample code
```python
import serial
import struct
ser = serial.Serial("/dev/ttyTHS1", 115200)

def getTFminiData():
    while True:
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)
            # check for header
            if recv[0] == 0x59 and recv[1] == 0x59:
                # Read high and low distance
                distance = recv[2] + (recv[3] << 8)
                # strength
                strength = recv[4] + (recv[5] << 8)
                print('(', distance, ',', strength, ')')
                # Calc checksum: 
                # Sum of the numbers of the first 8 bytes.
                lows = sum(recv[:-1])
                # Extract firt byte
                calc = struct.pack("I", lows)[0]
                # compare to checksum byte
                print(recv[8] == calc)
            else:
                #clear buffer
                ser.reset_input_buffer() 


if __name__ == '__main__':
    try:
        if not ser.is_open:
            ser.open()
        getTFminiData()
    except KeyboardInterrupt:   # Ctrl+C
        if ser != None:
            ser.close()
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [TFMini manual](https://cdn-shop.adafruit.com/product-files/3978/3978_manual_SJ-PM-TFmini-T-01_A03ProductManual_EN.pdf)
- [How to Convert Int to Bytes in Python 2 and Python 3](https://www.delftstack.com/howto/python/how-to-convert-int-to-bytes-in-python-2-and-python-3/)
- [nvida nano uart permission](https://forums.developer.nvidia.com/t/jetson-nano-how-to-use-uart-on-ttyths1/82037)
- [Jetson Nano – UART](https://www.jetsonhacks.com/2019/10/10/jetson-nano-uart/)