---
layout: post
title: CAN Bus hello
categories: [comm, hw]
tags: [can, hello-world]
public: true
---
<style>
img[src*='#size1'] {
    width: 200px;
    height: 200px;
}
</style>

# CAN bus
From wikipedia: A Controller Area Network is a robust serial communication bus found mostly in automotive and industrial environments design to allow microcontrollers and devices to communicate with each other's applications without a host computer. It is a message-based protocol.  
Device send the message sequentially if more than one device transmits at the same time the highest priority device is able to continue while the others back off. Frames are received by all devices, including by the transmitting device. 

# CAN 101
- Can controller
- Can Transceiver


![](/images/2020-05-02-07-39-19.png)

![](/images/2020-05-02-09-18-40.png)

## Differential signals

- Logic "1" recessive: recessive state is when the potential difference between CANH and CANL is 0V
- Logic "0" dominant: dominant state is when th potential difference between CANH and CANL is ~2V

![](/images/2020-05-02-09-33-37.png)

> ### **Dominant -> Logical 0 -> 2V diff**
&nbsp;  
&nbsp;  
&nbsp;  
## CAN Message
- Standard CAN (11 bit message-id)
- Extended CAN

### Message (Standard)
- SOF (1bit) Start Of Frame
- CAN-ID (11bit) 
- ROR (1 bit)
- IDE (1 bit) Standard or Extension msg 
- r0 - Reverse bit
- DLC - () Data length Code
- Data - (0-8 bytes)
- CRC (16 bit) - Checksum
- ACK (bit)
- EOF (7 bit)
- IFS


&nbsp;  
&nbsp;  
&nbsp;  
# Linux setup
```
sudo apt install can-utils
```

# inno-maker
USB to CAN interface

![](/images/2020-04-30-10-07-09.png#size1)

![](/images/2020-04-30-10-05-32.png#size1)



## Test
- Connect usb
- Set can interface
- Send data using `cangen`
- Receive data using `candump`
- and receiv data


### Setup
```bash
# device
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# virtual device
sudo ip link set vcan0 type can bitrate 500000
sudo ip link vcan0 up
```

### Send and Receive
```bash
# Terminal1 (can0)
candump can0

# Terminal2 (can1)
cansend can1 01a#11223344AABBCCDD

## Or generated data with 
cangen can1 -v

```
&nbsp;  
&nbsp;  
&nbsp;  
# Arduino
## sparkfun
- [ CAN-BUS Shield Hookup Guide ](https://learn.sparkfun.com/tutorials/can-bus-shield-hookup-guide/all)
  
![](/images/2020-04-30-15-27-38.png#size1)

> D9 wiring for OBD-II usage as default

&nbsp;  
&nbsp;  
# LAB
## Arduino ,RaspberryPI, CAN
Send and Receive CAN data between arduino and linux device  
using `Socketcan` with python library `python-can`

### HW
- RPi 3+
  - usb2can from inno-maker
- Arduino UNO
  - sparkfun CAN shield

### Wiring
- Shield default config is working with OBD2 device

| #     | RPi  | shield |
|-------|------|--------|
| CAN-H | Pin7 | Pin3   |
| CAN-L | Pin2 | Pin5   |

&nbsp;  
&nbsp;  
### Test1
- Linux Generated CAN data
  - using `can-utils` help utils
- Arduino Read and display data


#### RPi
```
cangen can0 -v
```

#### Arduino
[CAN-Bus sparkfun git ](https://github.com/sparkfun/CAN-Bus_Shield/tree/master/Libraries/Arduino/examples)
- Run `CAN_Read_Demo` example
&nbsp;  
&nbsp;  
&nbsp;  
### Test2
- Linux Read can data using python
- Arduino generated data using `CAN_Write_Demo`

#### Install python library
```
pip install python-can
```

#### Read all
```python
import can
from can.bus import BusState

def receive_all():
    """Receives all messages and prints them to the console until Ctrl+C is pressed."""
    with can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000) as bus:
        try:
            while True:
                msg = bus.recv(1)
                if msg is not None:
                    print(msg)

        except KeyboardInterrupt:
            pass  # exit normally


if __name__ == "__main__":
    receive_all()
```
&nbsp;  
&nbsp;  
&nbsp;  
### Test3 
- Python send one message example


```python
import can

def send_one():
    """Sends a single message."""
    with can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000) as bus:

        msg = can.Message(
            arbitration_id=0xC0FFEE, data=[0, 25, 0, 1, 3, 1, 4, 1], is_extended_id=True
        )

        try:
            bus.send(msg)
            print(f"Message sent on {bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")


if __name__ == "__main__":
    send_one()
```



&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Fastbit CAN Programing](https://youtu.be/i5iWAl74Iug?list=PLERTijJOmYrApVZqiI6gtA8hr1_6QS-cs)