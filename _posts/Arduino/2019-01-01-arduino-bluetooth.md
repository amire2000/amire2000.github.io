---
layout: post
title: Arduino Bluetooth
categories: Arduino
tags: [arduino, bluetooth]
---

- Arduino uno/nano
- HC-05 module (paring 1234)
- [Android](https://play.google.com/store/apps/details?id=com.giumig.apps.bluetoothserialmonitor&hl=en): Arduino bluetooth controller

## Arduino Uno
### Wiring

![](/images/2019-01-01-19-33-42.png)

| Arduino | HC-05  |
| ------- | ------ |
| +5 VDC  | VCC    |
| GND     | GND    |
| TX      | 10(RX) |
| RX      | 11(TX) |

### platformio.ini
```ini
[env:uno]
platform = atmelavr
framework = arduino
board = uno
upload_port=/dev/ttyUSB0
```

- Send `1` from BT Terminal mode to light **on** LED
- Send `0` from BT Terminal mode to light **off** LED
  
### Source
```cpp
#include <Arduino.h>
#include <SoftwareSerial.h>

#define TX 11
#define RX 10
#define BAUD 9600

SoftwareSerial bt_serial(RX, TX);
int bt_data; 

void setup()
{
  bt_serial.begin(BAUD);
  bt_serial.println("Bluetooth On please press 1 or 0 blink LED ");
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  if (bt_serial.available())
  {
    bt_data = bt_serial.read();
    if (bt_data == '1')
    { // if number 1 pressed ....
      digitalWrite(LED_BUILTIN, HIGH);
      bt_serial.println("LED  On ");
    }
    if (bt_data == '0')
    { // if number 0 pressed ....
      digitalWrite(LED_BUILTIN, LOW);
      bt_serial.println("LED  Off");
    }
  }
  delay(100);
}
```


## Arduino nano (not work with SoftwareSerial)

![](/images/2019-01-01-14-11-50.png)

## Reference
- [fritzing online](https://www.rollapp.com/app/fritzing)