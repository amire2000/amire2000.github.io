---
layout: post
title: 02 - GPIO Input, pull up and down 
categories: [hw, stm32]
tags: [stm32, arduino]
public: true
image: stm32icon.png
description: 
---

# Digital Input
Digital logic circuit have three logic state
- high
- low
- floating

![](/images/2020-06-06-00-42-19.png)
&nbsp;  
&nbsp;  
## GPIO Input Mode
GPIO has 3 digital input mode: 
- input with internal pull-up,  
- input with internal pull-down,  
- input floating.  
The logic voltage of STM32 is 3.3V, so the logic voltage for GPIO input pins are also 3.3V, but there are several pins that 5V tolerant.

![](/images/2020-06-06-00-45-35.png)

&nbsp;  
&nbsp;  
&nbsp;  
# Connect to 
## 5v tolerant pin
### Pull down
![](/images/2020-06-06-00-36-13.png)

- PB4 -> Resistor -> Ground
- Vss -> sw -> PB4
&nbsp;  
&nbsp;  
&nbsp;  
## Pull-up
![](/images/2020-06-06-00-36-32.png)




&nbsp;  
&nbsp;  
&nbsp;  
## Code example
```c
void setup() {
  pinMode(PC13, OUTPUT);
  pinMode(PB4, INPUT);
}


void loop() {
  if (digitalRead(PB4)){
    digitalWrite(PC13, HIGH);
  }
  else
  {
    digitalWrite(PC13, LOW);
  }
  delay(100);
}

```
&nbsp;  
&nbsp;  
&nbsp;  

# ST-Link
![](/images/2020-06-06-13-55-56.png)

## Blue pill
1. 3.3 v
2. SWDIO
3. SWCLK
4. GND

## STLink

```
```

&nbsp;  
&nbsp;  
&nbsp;  
# STM32CubeMX
STM32CubeMX is a graphical tool that allows a very easy configuration of STM32 microcontrollers and microprocessors, as well as the generation of the corresponding initialization C code for the Arm

## Install

### desktop / start menu

```
vim ~/.local/share/applications/stm32cubemx.desktop
```

```ini
[Desktop Entry]
Name=STM32CubeMX
GenericName=STM32CubeMX
Comment=Graphical tool for configuration of STM32 Microcontrollers
Exec=/home/user/ide/STM32CubeMX/STM32CubeMX
Terminal=false
X-MultipleArgs=false
Type=Application
Icon=/home/user/ide/STM32CubeMX/stm32.png
StartupWMClass=STM32CubeMX
StartupNotify=true
```

## Run
> Fix `Exception in thread "main" java.awt.AWTError: Assistive Technology not found: org.GNOME.Accessibility` 

- Mark `assistive_technologies=org.GNOME.Accessibility.AtkWrapper` line in `/etc/java-8-openjdk/accessibility.properties` file


```bash
sudo vim /etc/java-8-openjdk/accessibility.properties
```

![](/images/2020-06-06-00-18-24.png)



# References
- [STM32F103 SPL Tutorial 3 – GPIO Read Button](http://www.handsonembedded.com/stm32f103-spl-tutorial-3/)
- [java.awt.AWTError](https://blog.softhints.com/java-11-and-ubuntu-assistive-technology-not-found-awterror/)