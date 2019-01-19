---
layout: post
title: arduino timers
categories: arduino
tags: [arduino]
---
# Timers
Arduino UNO (Atml328p) has three Timers
- Timer0: 8 bit
- Timer1: 16 bit
- Timer2: 8 bit

## Example Timer1 (16bit)
![](/images/2019-01-16-22-19-18.png)

## Timer and pre scaler
Arduino  UNO has 16MHz clock, Timer can count each clock pulse or use prescaler.

- Without prescaler
$$
\frac{1}{16Mhz} =62.5ns
$$
- With prescaler(8)
$$
\frac{1*8}{16Mhz} =500ns
$$

### Possible Interrupts
- Compare Match
- Overflow
- Input Capture: Store it's  current value where external interrupt happen

### Demo: using interrupt to toggle led
#### Example without using interrupt
```cpp
const int LED_PIN = 13;

void setup(){
    pinMode(LED_PIN, OUTPUT);
}

void  loop(){
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}
```

- Using compare register to toggle LED every 500ms
- Using Timer1 (16bit) as a counter.
- Using pre scaler to setup compare value


$$
\frac{0.5s}{\frac{1}{16}}=\frac{(0.5sec*16MHz)}{1}=8,000,000 \newline
\frac{0.5s}{\frac{1}{16}}=\frac{(0.5sec*16MHz)}{8}=1,000,000 \newline
\frac{0.5s}{\frac{1}{16}}=\frac{(0.5sec*16MHz)}{64}=125,000 \newline
\frac{0.5s}{\frac{1}{16}}=\frac{(0.5sec*16MHz)}{256}=31,250 \newline
\frac{0.5s}{\frac{1}{16}}=\frac{(0.5sec*16MHz)}{1024}=7,812.5 
$$
> Select pre scaler 256:    
> Set CS12:1,  CS11:0 CS10:0 for timer 1

![](/images/2019-01-16-22-52-06.png)


## Arduino uno schematic
- Internal LED connect to port PB5
![](/images/2019-01-18-09-29-24.png)

- Using Register to toggle LED (section 11.4)
  - DDRB: set PB5 pin as output
  - PORTB: toggle PB5 on / off

```c
#include <Arduino.h>

#define LED PB5

void setup() {
    DDRB |= _BV(LED);
}

void loop() {
    PORTB ^= _BV(LED);
    delay(1000);     
}
```

## Timer registers
- TCNTx: Timer value
- TCCRx: Timer behavior (prescaler, ctc mode)
- OCRx: Compare Register
- TIMSKx: Interrupt mask register enable/disable timer interrupt
- ICRx: Input capture Register
- TIFRx: Timer interrupt Flag register


## Interrupt
- Internal interrupt: Timer (compare, overflow)
- External interrupt (reset,SPI, I2C)

### Enabling / disabling interrupts
```
sei();

cli();
```

## Timer compare interrupt example
```c
#include <Arduino.h>

#define LED PB5
const  uint16_t TIMER_LOAD = 0;
const  uint16_t COMPARE_VALUE = 31250;

void setup() {
    //Port B data direction
    DDRB |= _BV(LED);

    //Register behaviour
    TCCR1A = 0;
    TCCR1B = 0;
    //set prescaler 256
    TCCR1B |= _BV(CS12);
    //set ctc mode
    TCCR1B |= _BV(WGM12);
    //init timer and compare register
    TCNT1  = 0;
    OCR1A = COMPARE_VALUE;

    //enable timer interrupt
    TIMSK1 |= _BV(OCIE1A);

    sei();
}

//interrupt vector  TIMER1 + COMPARE Register + "vect"
ISR(TIMER1_COMPA_vect)
{
    PORTB ^= _BV(LED);
}

void loop() {
    delay(1000);     
}
```

## Reference
- [Level Up Your Arduino Code: Timer Interrupts](https://www.youtube.com/watch?v=2kr5A350H7E)
- [Atml 328 pdf](https://www.sparkfun.com/datasheets/Components/SMD/ATMega328.pdf)
- [Arduino 101: Timers and Interrupts](https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072)
- [How interrupt work in arduino uno](https://arduino.stackexchange.com/questions/30968/how-do-interrupts-work-on-the-arduino-uno-and-similar-boards)