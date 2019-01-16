---
layout: post
title: arduino registers
categories: arduino
tags: [arduino]
---

## Memory and registers
![](/images/2019-01-16-07-30-08.png)

> SFR: Special Function Register

##  Pins
![](/images/2019-01-16-08-47-15.png)
Pin 11 has the label PD5 (Port D pin 5)

### configure the pins (Avr  manual 14.2.1)

#### DDRD: Data Direction Register (port D)
![](/images/2019-01-16-08-51-51.png)
- 1: make the pin output
- 0" make the pin input
  
#### PORTD: Data Register (Port D)
![](/images/2019-01-16-08-54-21.png)
- if pin is **output**
  - 1: HIGH
  - 0: LOW
- if pin is **input**
  - 1: enabled pull up resistor
  - 0: disabled pull up resistor
### PIND: Pin Address (port D) 
![](/images/2019-01-16-08-57-51.png)

### Register Address
> Reserve Register used from internal cpu processing and  should  not be  touch

> Manual section 36

# Examples
## Standard Arduino code (Sketch uses 902 bytes)
```cpp
const int btn_pin = 2;
const int led_pin = 5;

void setup(){
    pinMode(btn_pin, INPUT_PULLUP);
    pinMode(led_pin, OUTPUT);
}

void  loop(){
    int btn = digitalRead(btn_pin);

    if (btn == LOW){
        digitalWrite(led_pin, HIGH);
    }else{
        digitalWrite(led_pin, LOW);
    }
}
```

## Using registers
Pin (2,5) are belong to register Port D
![](/images/2019-01-16-09-26-15.png)

#### PortD Registers
-  DDRD (bit5 as output) 0b00100000
-  PORTD (bit2 as pullup resistor) 0b00000100
> PIND, DDRD and PORTD are macros define in `iom328p.h` file 
#### Read values from PIND  register
Read `bit 2` value
- Zero all bit's except bit2 with AND operation
- Shift Right the result by 2
- check value in `bit 0` 
  ```
  tmp =  PIND & (1 << 2)
  result =  tmp >> 2
  ```
## Use registers (Sketch uses 462 bytes)
```cpp
const int btn_pin = 2;
const int led_pin = 5;

void setup(){
    DDRD = B00100000;
    PORTD = B00000100;
}

void  loop(){
    int btn = (PIND & (1 << btn_pin)) >> btn_pin;

    if (btn == LOW){
        PORTD = (1 << led_pin)  |  PORTD;
    }else{
        PORTD = ~(1  << led_pin) & PORTD;
    }
}
```

> Using the _BV  macro from sfr_defs.h  
> `#define _BV(bit) (1 << (bit))`


## reference
- [Level Up Your Arduino Code: Registers](https://www.youtube.com/watch?v=6q1yEb_ukw8)