---
layout: post
title: arduino PID
categories: arduino
tags: [arduino, PID]
---

PID is a control loop feedback system.  
PID is  short for  Proportional, Integral and Derivative

## Proportional
Adjustment are proportional to how mush the `error` is.

## Integral
Produce an adjustment that is based on the accumulated error over time.

## Derivative
Produce an adjustment to deal with error rate of change.

![](/images/2019-01-19-17-15-11.png)

# PID Implementation
- PID computation must be inside a looping function
- PID parameters
  - P
  - I
  - D
  - Set point
  - Input value (feedback)

```c
//PID constants
double kp = 2
double ki = 5
double kd = 1

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

void setup(){}    

void loop(){
        input = analogRead(A0);                //read from rotary encoder connected to A0
        output = computePID(input);
        delay(100);
        analogWrite(3, output);                //control the motor based on PID value

}

double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = Setpoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative

        double out = kp*error + ki*cumError + kd*rateError;                //PID output               

        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        return out;                                        //have function return the PID output
}
```

## Reference
- [Arduino PID Control Tutorial](https://www.teachmemicro.com/arduino-pid-control-tutorial/)