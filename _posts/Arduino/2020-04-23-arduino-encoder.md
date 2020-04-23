---
layout: post
title: Control DC Motor speed with arduino and rotary encoder
categories: arduino
tags: [encoder]
public: true
---
# LAB
- Encoder
- DC Motor and H-Bridge
- Arduino PID 

&nbsp;  
&nbsp;  
&nbsp;  

# Encoder
## Encoder Theory
The KY-040 rotary encoder is a rotary input device that provides an indication of how much the knob has been rotated AND what direction it is rotating in.

> Check `Henry` blog. link at reference section
  
&nbsp;  
&nbsp;  
&nbsp;  
## Wiring
- CLK: Encoder Pin A
- DT: Encoder Pin B
- SW: Pushbotton
- +: Vcc (5v)
- GND: GND (Pin C)


![](/images/Keyes-KY-040-arduino-wiring.png)

&nbsp;  
&nbsp;  
&nbsp;  
## Code
```cpp
//henrysbench.capnfatz.com
 int pinA = 3;  // Connected to CLK on KY-040
 int pinB = 4;  // Connected to DT on KY-040
 int encoderPosCount = 0; 
 int pinALast;  
 int aVal;
 boolean bCW;

 void setup() { 
   pinMode (pinA,INPUT);
   pinMode (pinB,INPUT);
   /* Read Pin A
   Whatever state it's in will reflect the last position   
   */
   pinALast = digitalRead(pinA);   
   Serial.begin (9600);
 } 

 void loop() { 
   aVal = digitalRead(pinA);
   if (aVal != pinALast){ // Means the knob is rotating
     // if the knob is rotating, we need to determine direction
     // We do that by reading pin B.
     if (digitalRead(pinB) != aVal) {  // Means pin A Changed first - We're Rotating Clockwise
       encoderPosCount ++;
       bCW = true;
     } else {// Otherwise B changed first and we're moving CCW
       bCW = false;
       encoderPosCount--;
     }
     Serial.print ("Rotated: ");
     if (bCW){
       Serial.println ("clockwise");
     }else{
       Serial.println("counterclockwise");
     }
     Serial.print("Encoder Position: ");
     Serial.println(encoderPosCount);
     
   } 
   pinALast = aVal;
 } 
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Keyes KY-040 Arduino Rotary Encoder User Manual](http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/keyes-ky-040-arduino-rotary-encoder-user-manual/)