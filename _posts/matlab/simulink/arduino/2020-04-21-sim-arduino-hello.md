---
layout: post
title: Simulink and Arduino Hello
categories: simulink
tags: [arduino]
public: true
description: Connect arduino with simulink
image: sim_arduino.png
---

# Download And Install
Install from matlab `Add on` or manually from web
## Download from web
![](/images/2020-04-21-12-04-50.png)
- Download From matlab site 
  - [Arduino package](https://www.mathworks.com/matlabcentral/fileexchange/40312-simulink-support-package-for-arduino-hardware)
- Drag file to command window and run
  
![](/images/2020-04-21-11-37-51.png)


# Setup
- Configure module
&nbsp;  
&nbsp;  
### Hardware Implementation
- Select board
- Select `Host board connection` automatically or manual

![](/images/2020-04-21-12-19-48.png)
&nbsp;  
&nbsp;  
### Solver
- Stop time: Inf
- Type: Fixed step
- Fixed step size: 0.01

![](/images/2020-04-21-12-22-41.png)

&nbsp;  
&nbsp;  
&nbsp;  
# Simple Module
Arduino hello world blinking LED


- Add arduino `Digital output`
  - Set pin to internal LED `pin 13`
- Add Puls Generator
  - Set interval
  - Set Puls wide
- Set Simulator mode to `External`
  
![](/images/2020-04-21-14-17-09.png)

&nbsp;  
&nbsp;  
&nbsp;  
## Module
### Digital output
- Blink internal LED


![](/images/2020-04-21-14-13-18.png)

&nbsp;  
&nbsp;  
&nbsp;  
### Analog Input
- Read value from potentiometer

![](/images/2020-04-22-22-52-55.png)

&nbsp;  
&nbsp;  
&nbsp;  
### Digital input
- Read switch state
  - Pull-up
  - Pull-down

![](/images/2020-04-22-22-57-28.png)

## TTL level ant other
- 5V TTL
- 3.3 TTL


[Logic Levels ](https://learn.sparkfun.com/tutorials/logic-levels/ttl-logic-levels)

![](/images/2020-04-22-23-05-06.png)
- image from [sparkfun](https://learn.sparkfun.com/tutorials/logic-levels/ttl-logic-levels)
