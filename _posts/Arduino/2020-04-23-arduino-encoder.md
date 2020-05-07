---
layout: post
title: Control DC Motor speed with arduino and rotary encoder
categories: arduino
tags: [encoder]
public: true
---
# LAB
- [Encoder](#encoder)
- [DC Motor and H-Bridge](#dc-motors-and-h-bridge)
- [Arduino PID controller](#arduino-pid-controller)

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
# DC-Motors and H-Bridge
An H-bridge is built of four switches that control the flow of current to a load

![](/images/2020-04-24-00-43-01.png)

![](/images/2020-04-24-00-43-37.png)
images from digilentinc.com
&nbsp;  
&nbsp;  
## L298N
The L298N is a dual H-Bridge motor driver which allows speed and direction control of two DC motors at the same time. The module can drive DC motors that have voltages between 5 and 35V, with a peak current up to 2A.
![](/images/2020-04-24-00-39-32.png)

## Wire
> Power supply 12 vdc (5v occur some undefined behavior)

> Share `GND` between Bridge and Arduino


- 3 -> enb
- 5 -> in3
- 4 -> in4

## Code
- Control speed by using enable pin
  
```c
int enB = 3;
int in3 = 5;
int in4 = 4;

void setup() {

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  speedControl();
  delay(1000);
}

void speedControl() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  for (int i = 30; i < 100; i++) {
    analogWrite(enB, i);
    delay(200);
  }

}
```

&nbsp;  
&nbsp;  
&nbsp;  
# Arduino PID Controller

&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Keyes KY-040 Arduino Rotary Encoder User Manual](http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/keyes-ky-040-arduino-rotary-encoder-user-manual/)