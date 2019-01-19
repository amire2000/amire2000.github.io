---
layout: post
title: Arduino and VSCode
categories: Arduino
tags: [arduino, vscode]
---
![](/images/2019-01-18-08-42-04.png)
The Arduino extension makes it easy to develop, build, deploy and debug your Arduino sketches in Visual Studio Code  
The extension require that arduino IDE installed (I use version 1.8.8) 

## Setup arduino project
- Open folder
- From command pallet `Arduino: initialize"
  - Select arduino board
> The init process create  
>   .vscode: arduino.json  
>   .vscode:c_cpp_properties.json  
>   .vscode: settings.json

- Select Arduino port from status bar or add to `arduino.json`
```
{
    "sketch": "app.ino",
    "board": "arduino:avr:uno",
    "port": "/dev/ttyUSB0"
}
```
- Add /  update `C_Cpp.intelliSenseEngine` in settings.json (see note)
```
{
    "C_Cpp.intelliSenseEngine": "Tag Parser"
}
```
## src/main.cpp
```c
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
- Arduino: Verify 
- Arduino: Upload (Ctrl+Alt+U)

## Reference
- [IntelliSense engines](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/IntelliSense%20engine.md)
## Notes
* Without update `intelliSenseEngine` the vscode 



