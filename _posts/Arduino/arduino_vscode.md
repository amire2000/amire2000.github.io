---
layout: post
title: Arduino and VSCode
categories: Arduino
tags: [arduino, vscode]
---
VSCode setup for arduino coding and other IoT devices

# Install vscode extension
- PlatformIO IDE: Development environment for IoT, Arduino, ARM mbed, Espressif 
- Arduino: Arduino for visual studio code depend on Arduino IDE
![](/images/2019-01-01-12-30-53.png)

## Create a new project (Arduino nano)
- From PIO Home
    - select board 328 not like in the picture
![](/images/2019-01-01-12-35-33.png)

- PlatformIO create project files
  - platformio.ini
  
## platformio.ini
- Fix upload_port
- Fix/Select board

```ini
[env:nanoatmega328]
platform = atmelavr
board = nanoatmega328
framework = arduino
upload_port=/dev/ttyUSB0
```

## src/main.cpp
```cpp
#include <Arduino.h>

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                       // wait for a second
}
```

## Build and upload
From Command palette
- PlatformIO: build (Ctrl+Alt+B)
- PlatformIO: Upload (Ctrl+Alt+U)

## Reference
- [PlatformIO support boards](https://platformio.org/boards)
- [PlatformIO ini file](https://docs.platformio.org/en/latest/projectconf.html)

