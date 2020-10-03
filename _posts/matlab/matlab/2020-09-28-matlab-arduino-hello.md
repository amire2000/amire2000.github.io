---
layout: post
title: MATLAB and Arduino Hello
categories: matlab
tags: [arduino]
public: true
description: Connect arduino with MATLAB
image: arduino.png
---

# Download and Install
MATLAB® Support Package for Arduino® Hardware enables you to use MATLAB to communicate with an Arduino board.
- Download
[MATLAB Support Package for Arduino Hardware](https://www.mathworks.com/matlabcentral/fileexchange/47522-matlab-support-package-for-arduino-hardware)
- install
Drag and Drop to `arduinoio.mlpkginstall` to matlab

# First code
## Blink LED
- Board: Uno
- Internal LED (D13)
  
```
a = arduino('/dev/ttyUSB0', 'Uno');
for i = 1:10
    writeDigitalPin(a, 'D13', 0);
    pause(0.5);
    writeDigitalPin(a, 'D13', 1);
    pause(0.5);
end
```

## I2C
- Wire MPU 9250
  
### Find Devices
```
a = arduino('/dev/ttyUSB0','Uno','Libraries','I2C');
scanI2CBus(a,0)

ans = 1×1 cell array
    {'0x68'}
```

### Connect and read
```cpp
// R2018 command
obj = i2cdev(a, "0x68")
write
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Getting Started with MATLAB Support Package for Arduino Hardware](https://www.mathworks.com/help/supportpkg/arduinoio/examples/getting-started-with-matlab-support-package-for-arduino-hardware.html)
- [I2C Devices](https://www.mathworks.com/help/releases/R2018b/supportpkg/arduinoio/i2c-sensors.html)
