---
layout: post
title: Arduino and VSCode
categories: Arduino
tags: [arduino, vscode]
image: 2019-01-18-08-42-04.png
public: true
description: Install setup and usage Arduino vscode extension
---
# Content
- Install Extension
- Init Arduino VSCode extension
- c/cpp plugin tips
 
# Install
![](/images/2019-05-29-22-52-50.png)

The Arduino extension makes it easy to develop, build, deploy and debug your Arduino sketches in Visual Studio Code  
> The extension require that arduino IDE installed (I use version 1.8.9) 

## settings
- Arduino: Path `/home/user/ide/arduino-1.8.9`
> Restart after set

# Setup arduino project
- Create new project 
- From command pallet `Arduino: initialize`
  - Select arduino board
- The init process create  
  - .vscode/arduino.json  
  - .vscode/c_cpp_properties.json  
  - .vscode/settings.json

- Arduino board and port selection from VSCode status bar
  
![](/images/2019-05-29-23-27-46.png)

## `arduino.json`
```
{
    "sketch": "app.ino",
    "board": "arduino:avr:mega",
    "configuration": "cpu=atmega2560",
    "port": "/dev/ttyACM0"
}
```

## `c_cpp_properties.json` 
- Add include to remove `<Arduino.h>` and dependencies warnings
- Add `USBCON` to remove `Serial` undefined warning
```
{
"includePath": [
  "${workspaceFolder}/**",
  "/home/user/ide/arduino-1.8.9/tools/**",
  "/home/user/ide/arduino-1.8.9/hardware/arduino/avr/**",
  "/home/user/ide/arduino-1.8.9/hardware/arduino/avr/cores/arduino/**",
  "/home/user/ide/arduino-1.8.9/hardware/tools/avr/avr/include/**",
  "/home/user/ide/arduino-1.8.9/hardware/arduino/avr/variants/mega/**"
],
"defines": [
  "USBCON"
]
}
```
## app.ino
```c
#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("turn");
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
```

## Build and upload
From Command palette
- Arduino: Verify 
- Arduino: Upload (Ctrl+Alt+U)

## Reference
- [IntelliSense engines](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/IntelliSense%20engine.md)




